// Residual (error) dynamics via local weighted linear regression — C++ port
// of online_training/residual_learnt_dynamics/error_regression.py, which in
// turn transcribes the math of Racing-LMPC-ROS2 safe_set.cpp
// (SSTrajectory::query(RegQuery) / SafeSetManager::query).
//
// Port contract: given the same buffer and queries, predictions must match
// the numpy implementation to floating-point noise (verified by
// online_training/verify_cpp_port.py). Hence:
//   * neighbor search is brute force over scaled features — the same
//     neighbor set scipy.cKDTree returns (metric identical, no approximation),
//   * feature scales are the population std (numpy .std(), ddof=0),
//   * the local solve is a partial-pivot LU (numpy.linalg.solve / LAPACK).
//
// The nominal model is the discrete kinematic bicycle
// (nominal_dynamics/kinematic_bicycle.py):
//   v+ = |v| + a Ts, beta+ = atan(l_r tan(delta)/L),
//   vx+ = v+ cos(beta+), vy+ = v+ sin(beta+), omega+ = v+ sin(beta+)/l_r.

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace residual {

constexpr int NX = 3;              // vx, vy, omega
constexpr int NU = 2;              // a, delta
constexpr int NZ = NX + NU;        // feature dim
constexpr int NM = NZ + 1;         // regressors [x, u, 1]

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix;

struct KinematicBicycleNominal {
  double l_f, l_r, L, Ts;
  KinematicBicycleNominal(double lf, double lr, double ts)
  : l_f(lf), l_r(lr), L(lf + lr), Ts(ts) {}

  inline Eigen::Vector3d predict(const Eigen::Vector3d& x,
                                 const Eigen::Vector2d& u) const {
    const double v = std::hypot(x(0), x(1));
    const double v1 = v + u(0) * Ts;
    const double b1 = std::atan(l_r * std::tan(u(1)) / L);
    return {v1 * std::cos(b1), v1 * std::sin(b1), v1 * std::sin(b1) / l_r};
  }
};

class ErrorDynamicsRegressor {
public:
  // max_samples: 0 = unlimited (offline default). > 0 = ring buffer that
  // overwrites the oldest samples — the online mode's equivalent of
  // Racing-LMPC-ROS2's circular_buffer(max_lap_stored); keeps the per-solve
  // neighbor scan and the per-step buffer bookkeeping O(max_samples).
  ErrorDynamicsRegressor(double l_f, double l_r, double Ts,
                         double dist_max = 2.0, int k_max = 256,
                         double ridge = 1e-3, int min_pts = 10,
                         bool std_scale = true, int max_samples = 0)
  : nominal_(l_f, l_r, Ts), h_(dist_max), k_max_(k_max), ridge_(ridge),
    min_pts_(min_pts), std_scale_(std_scale), max_samples_(max_samples) {
    if (max_samples_ > 0) {
      M_.resize(max_samples_, NM);
      E_.resize(max_samples_, NX);
    }
  }

  // X, X_next: (n, 3); U: (n, 2). Row i of X_next measured Ts after (X_i, U_i).
  void add_samples(const MatrixXd& X, const MatrixXd& U,
                   const MatrixXd& X_next) {
    if (X.cols() != NX || U.cols() != NU || X_next.cols() != NX ||
        X.rows() != U.rows() || X.rows() != X_next.rows())
      throw std::invalid_argument("add_samples: bad shapes");
    const auto n = X.rows();
    if (max_samples_ == 0) {
      M_.conservativeResize(n_rows_ + n, NM);
      E_.conservativeResize(n_rows_ + n, NX);
    }
    for (Eigen::Index i = 0; i < n; ++i) {
      Eigen::Index r;
      if (max_samples_ > 0) {
        r = write_;
        write_ = (write_ + 1) % max_samples_;
        if (n_rows_ < max_samples_) n_rows_++;
      } else {
        r = n_rows_++;
      }
      M_.row(r) << X(i, 0), X(i, 1), X(i, 2), U(i, 0), U(i, 1), 1.0;
      const Eigen::Vector3d xn = nominal_.predict(
          X.row(i).head<NX>(), U.row(i).head<NU>());
      E_.row(r) = X_next.row(i) - xn.transpose();
    }
    built_ = false;
  }

  Eigen::Index n_samples() const { return n_rows_; }

  // Batch one-step prediction. Returns (n,3) predictions; used[i] = 1 where
  // the local regression had >= min_pts neighbors (pure nominal elsewhere).
  std::pair<MatrixXd, std::vector<uint8_t>> predict(const MatrixXd& X,
                                                    const MatrixXd& U) {
    build();
    const Eigen::Index n = X.rows();
    MatrixXd out(n, NX);
    std::vector<uint8_t> used(n, 0);
    for (Eigen::Index i = 0; i < n; ++i) {
      const Eigen::Vector3d x = X.row(i).head<NX>();
      const Eigen::Vector2d u = U.row(i).head<NU>();
      Eigen::Vector3d pred = nominal_.predict(x, u);
      Matrix<double, NM, NX> theta;
      if (local_theta(x, u, theta)) {
        Matrix<double, 1, NM> mq;
        mq << x(0), x(1), x(2), u(0), u(1), 1.0;
        pred += (mq * theta).transpose();
        used[i] = 1;
      }
      out.row(i) = pred.transpose();
    }
    return {out, used};
  }

  // MPC interface: corrections to ADD to the nominal discrete A, B, C of the
  // velocity rows. Zeros when the neighborhood is too thin.
  bool linearize(const Eigen::Vector3d& x, const Eigen::Vector2d& u,
                 Eigen::Matrix3d& dA, Matrix<double, 3, 2>& dB,
                 Eigen::Vector3d& dC) {
    build();
    Matrix<double, NM, NX> theta;
    if (!local_theta(x, u, theta)) {
      dA.setZero(); dB.setZero(); dC.setZero();
      return false;
    }
    const Matrix<double, NX, NM> T = theta.transpose();
    dA = T.leftCols<NX>();
    dB = T.middleCols<NU>(NX);
    dC = T.rightCols<1>();
    return true;
  }

  const KinematicBicycleNominal& nominal() const { return nominal_; }

  // Full learnt discrete velocity map for the MPC: prediction and Jacobians
  // of  w+ = f_nom(w, u) + [dA dB dC]·[w; u; 1]  in w = (v_x, v_y, omega)
  // space.  Gw = d w+/d w,  Gu = d w+/d u.  Works with an empty/thin buffer
  // (falls back to pure nominal; returns whether regression contributed).
  bool linearize_velocity(const Eigen::Vector3d& w, const Eigen::Vector2d& u,
                          Eigen::Matrix3d& Gw, Matrix<double, 3, 2>& Gu,
                          Eigen::Vector3d& w_next) {
    // nominal prediction and analytic Jacobians
    const double Ts = nominal_.Ts, l_r = nominal_.l_r, L = nominal_.L;
    const double v = std::max(std::hypot(w(0), w(1)), 1e-3);
    const double v1 = std::hypot(w(0), w(1)) + u(0) * Ts;
    const double tb = l_r * std::tan(u(1)) / L;
    const double b1 = std::atan(tb);
    const double cb = std::cos(b1), sb = std::sin(b1);
    w_next << v1 * cb, v1 * sb, v1 * sb / l_r;

    // d w+/d w : only through v = |(vx, vy)|
    const double dv_dvx = w(0) / v, dv_dvy = w(1) / v;
    Gw << cb * dv_dvx,        cb * dv_dvy,        0.0,
          sb * dv_dvx,        sb * dv_dvy,        0.0,
          sb / l_r * dv_dvx,  sb / l_r * dv_dvy,  0.0;
    // d w+/d u : a through v1; delta through beta1
    const double sec2 = 1.0 + std::tan(u(1)) * std::tan(u(1));
    const double db_dd = (l_r / L) * sec2 / (1.0 + tb * tb);
    Gu << Ts * cb,        -v1 * sb * db_dd,
          Ts * sb,         v1 * cb * db_dd,
          Ts * sb / l_r,   v1 * cb / l_r * db_dd;

    if (n_rows_ < min_pts_) return false;
    build();
    Matrix<double, NM, NX> theta;
    if (!local_theta(w, u, theta)) return false;
    const Matrix<double, NX, NM> T = theta.transpose();
    Gw += T.leftCols<NX>();
    Gu += T.middleCols<NU>(NX);
    Matrix<double, NM, 1> mq;
    mq << w(0), w(1), w(2), u(0), u(1), 1.0;
    w_next += T * mq;
    return true;
  }

private:
  void build() {
    if (built_) return;
    if (n_rows_ == 0) throw std::runtime_error("empty buffer");
    const auto Mv = M_.topRows(n_rows_);   // valid rows (ring may overwrite)
    scales_.setOnes();
    if (std_scale_) {
      for (int j = 0; j < NZ; ++j) {
        const auto col = Mv.col(j);
        const double mean = col.mean();
        const double var = (col.array() - mean).square().mean();  // ddof=0
        const double s = std::sqrt(var);
        scales_(j) = (s > 1e-8) ? s : 1.0;
      }
    }
    Zs_ = Mv.leftCols<NZ>().array().rowwise() /
          scales_.transpose().array();
    built_ = true;
  }

  // Weighted ridge LS around one query point; false if < min_pts neighbors.
  bool local_theta(const Eigen::Vector3d& x, const Eigen::Vector2d& u,
                   Matrix<double, NM, NX>& theta) {
    Matrix<double, 1, NZ> zq;
    zq << x(0), x(1), x(2), u(0), u(1);
    zq.array() /= scales_.transpose().array();

    // brute-force squared distances, keep those within h (as cKDTree with
    // distance_upper_bound), then the k_max nearest of them
    const double h2 = h_ * h_;
    dist_idx_.clear();
    for (Eigen::Index r = 0; r < Zs_.rows(); ++r) {
      const double d2 = (Zs_.row(r) - zq).squaredNorm();
      if (d2 <= h2) dist_idx_.emplace_back(d2, r);
    }
    if ((int)dist_idx_.size() < min_pts_) return false;
    const size_t k = std::min<size_t>(k_max_, dist_idx_.size());
    std::partial_sort(dist_idx_.begin(), dist_idx_.begin() + k,
                      dist_idx_.end());

    Matrix<double, NM, NM> Q = ridge_ * Matrix<double, NM, NM>::Identity();
    Matrix<double, NM, NX> b = Matrix<double, NM, NX>::Zero();
    for (size_t j = 0; j < k; ++j) {
      const double d = std::sqrt(dist_idx_[j].first);
      const double t = 1.0 - (d / h_) * (d / h_);
      const double w = 0.75 / h_ * t * t;             // Epanechnikov-style
      const auto m = M_.row(dist_idx_[j].second);
      Q.noalias() += w * m.transpose() * m;
      b.noalias() += w * m.transpose() * E_.row(dist_idx_[j].second);
    }
    theta = Q.partialPivLu().solve(b);                // = numpy.linalg.solve
    return true;
  }

  KinematicBicycleNominal nominal_;
  double h_;
  int k_max_;
  double ridge_;
  int min_pts_;
  bool std_scale_;
  int max_samples_;

  MatrixXd M_;   // (n, 6) regressors [x, u, 1]
  MatrixXd E_;   // (n, 3) one-step errors vs nominal
  MatrixXd Zs_;  // (n_rows_, 5) scaled features of the valid rows
  Matrix<double, NZ, 1> scales_;
  Eigen::Index n_rows_ = 0;   // valid samples
  Eigen::Index write_ = 0;    // ring write cursor (max_samples_ > 0)
  bool built_ = false;
  std::vector<std::pair<double, Eigen::Index>> dist_idx_;  // scratch
};

}  // namespace residual
