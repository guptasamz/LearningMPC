"""Residual (error) dynamics via local weighted linear regression.

Math transcribed from Racing-LMPC-ROS2 (HaoruXue, branch humble),
src/vehicle_dynamics_models/racing_trajectory/src/safe_set.cpp,
SSTrajectory::query(RegQuery) + SafeSetManager::query(RegQuery):

  * lazy learner: store (x_k, u_k, x_{k+1}) samples, no training phase
  * per query point z* = (x, u): neighbors with ||z - z*|| < dist_max,
    Epanechnikov-style weights  w = 0.75/h * (1 - (d/h)^2)^2
  * per output state: weighted ridge least squares of the one-step ERROR
        e = x_{k+1}^measured - f_nominal(x_k, u_k)
    on M = [x_sel, u_sel, 1]:   theta = (M^T W M + 1e-3 I)^{-1} M^T W e
  * learnt prediction: x+ = f_nominal(x, u) + [theta_x; theta_u; theta_1]·z*
    (equivalently corrections dA, dB, dC added to the nominal A, B, C)

Two deliberate deviations from their code, both flagged:
  1. Feature scaling. Their distance is Euclidean on raw (x, u) — with our
     units the steering column (|delta| <= 0.41) would be invisible next to
     v_x (0..8). We scale each feature by its buffer std before distance
     computation (dist_max is then in scaled units). scale=None reproduces
     their raw behavior.
  2. Sign. Their solver uses b = -M^T W y and then ADDS the result; we solve
     the plain normal equations and add. Offline MSE verifies our convention.
"""

import numpy as np
from scipy.spatial import cKDTree


class ErrorDynamicsRegressor:
    def __init__(self, nominal, dist_max=1.0, k_max=256, ridge=1e-3,
                 min_pts=10, scale="std"):
        """nominal: object with predict_velocities_stacked(X, U) -> (n,3).
        dist_max: neighbor radius = kernel bandwidth (scaled units).
        k_max: cap on neighbors per query (nearest first).
        scale: "std" = per-feature std scaling; None = raw units (their code).
        """
        self.nominal = nominal
        self.h = float(dist_max)
        self.k_max = int(k_max)
        self.ridge = float(ridge)
        self.min_pts = int(min_pts)
        self.scale_mode = scale
        self._X = []  # list of (n,3) blocks
        self._U = []
        self._E = []  # one-step errors vs nominal, (n,3)
        self._tree = None

    # ---------------- buffer ----------------
    def add_samples(self, X, U, X_next):
        """X, X_next: (n,3) [vx, vy, omega]; U: (n,2) [a, delta].
        Rows must be one-step pairs: X_next[i] measured Ts after (X[i], U[i])."""
        E = X_next - self.nominal.predict_velocities_stacked(X, U)
        self._X.append(np.asarray(X, float))
        self._U.append(np.asarray(U, float))
        self._E.append(E)
        self._tree = None  # invalidate

    @property
    def n_samples(self):
        return sum(b.shape[0] for b in self._X)

    def _build(self):
        X = np.vstack(self._X)
        U = np.vstack(self._U)
        self._Xb, self._Ub = X, U
        self._Eb = np.vstack(self._E)
        Z = np.hstack([X, U])
        if self.scale_mode == "std":
            s = Z.std(axis=0)
            self._scales = np.where(s > 1e-8, s, 1.0)
        else:
            self._scales = np.ones(Z.shape[1])
        self._Zs = Z / self._scales
        self._M = np.hstack([X, U, np.ones((len(X), 1))])  # regressors
        self._tree = cKDTree(self._Zs)

    # ---------------- query ----------------
    def _local_theta(self, zq_scaled, mq):
        """Weighted ridge LS around one query. Returns theta (6,3) or None."""
        d, idx = self._tree.query(zq_scaled, k=min(self.k_max, len(self._Zs)),
                                  distance_upper_bound=self.h)
        good = np.isfinite(d)
        if good.sum() < self.min_pts:
            return None
        d, idx = d[good], idx[good]
        w = 0.75 / self.h * (1.0 - (d / self.h) ** 2) ** 2
        M = self._M[idx]           # (k, 6)
        Y = self._Eb[idx]          # (k, 3)
        MW = M * w[:, None]
        Q = M.T @ MW + self.ridge * np.eye(M.shape[1])
        theta = np.linalg.solve(Q, MW.T @ Y)   # (6, 3)
        return theta

    def predict(self, X, U):
        """Learnt one-step prediction for queries X (n,3), U (n,2).
        Returns (X_next_pred (n,3), used_regression (n,) bool).
        Falls back to pure nominal where fewer than min_pts neighbors."""
        if self._tree is None:
            self._build()
        Xn = self.nominal.predict_velocities_stacked(X, U)
        Z = np.hstack([X, U])
        Zs = Z / self._scales
        Mq = np.hstack([Z, np.ones((len(X), 1))])
        used = np.zeros(len(X), bool)
        # one batched neighbor query for all points, then per-point solves
        k = min(self.k_max, len(self._Zs))
        D, I = self._tree.query(Zs, k=k, distance_upper_bound=self.h,
                                workers=-1)
        eye = self.ridge * np.eye(self._M.shape[1])
        for i in range(len(X)):
            good = np.isfinite(D[i])
            if good.sum() < self.min_pts:
                continue
            d, idx = D[i][good], I[i][good]
            w = 0.75 / self.h * (1.0 - (d / self.h) ** 2) ** 2
            M = self._M[idx]
            MW = M * w[:, None]
            theta = np.linalg.solve(M.T @ MW + eye, MW.T @ self._Eb[idx])
            Xn[i] += Mq[i] @ theta
            used[i] = True
        return Xn, used

    def linearize(self, x, u):
        """MPC interface: local corrections for one operating point.
        Returns (dA (3,3), dB (3,2), dC (3,)) to ADD to the nominal
        discrete-time A, B, C of the velocity rows; zeros if no data."""
        if self._tree is None:
            self._build()
        z = np.concatenate([x, u])
        theta = self._local_theta(z / self._scales, np.append(z, 1.0))
        if theta is None:
            return np.zeros((3, 3)), np.zeros((3, 2)), np.zeros(3)
        T = theta.T  # (3, 6): rows = output states
        return T[:, :3], T[:, 3:5], T[:, 5]
