// Vendored verbatim from f1tenth_ws/src/syn_pf_cpp (ForzaETH/particle_filter,
// restructured to ROS2 -- see UPSTREAMS.md there), used here to run a real
// particle filter against the simulator's own lidar/map/odom, matching
// f1tenth_ws's own run_verify_real_in_sim.sh pattern -- so a lmpc_ros2 sim
// run exercises the SAME pose_source=pf state-reconstruction path
// (lmpc_node's reconstruct_from_pf()) the real car uses, not sim ground
// truth. One local change from upstream: build_range_method()'s "rmgpu"
// branch is now #ifdef USE_CUDA-guarded (was unconditional) -- range_libc's
// RayMarchingGPU doesn't declare set_sensor_model() without CUDA, which
// broke compilation entirely on this CPU-only Docker image. Never selected
// at runtime here anyway (see ../config/synpf_cpp_params.yaml's
// range_method: cddt).
#include "syn_pf_cpp/synpf_component.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace syn_pf_cpp {

SynPFComponent::SynPFComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("synpf_cpp", options)
{
  pose_pub_topic_ = declare_parameter<std::string>("pose_pub_topic", "/tracked_pose");
  scan_topic_     = declare_parameter<std::string>("scan_topic", "/scan");
  odom_topic_     = declare_parameter<std::string>("odometry_topic", "/odom");
  max_particles_  = declare_parameter<int>("max_particles", 3000);
  update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);
  trigger_on_scan_ = declare_parameter<bool>("trigger_on_scan", false);

  init_var_x_  = declare_parameter<double>("initial_var_x", 0.5);
  init_var_y_  = declare_parameter<double>("initial_var_y", 0.5);
  init_var_th_ = declare_parameter<double>("initial_var_theta", 0.4);

  alpha1_     = declare_parameter<double>("alpha_1_tum", 0.5);
  alpha2_     = declare_parameter<double>("alpha_2_tum", 0.015);
  alpha3_     = declare_parameter<double>("alpha_3_tum", 1.0);
  alpha4_     = declare_parameter<double>("alpha_4_tum", 0.1);
  lam_thresh_ = declare_parameter<double>("lam_thresh", 0.1);

  use_initial_pose_ = declare_parameter<bool>("use_initial_pose", false);
  initial_x_        = declare_parameter<double>("initial_pose_x", 0.0);
  initial_y_        = declare_parameter<double>("initial_pose_y", 0.0);
  initial_theta_    = declare_parameter<double>("initial_pose_theta", 0.0);
  rng_seed_         = declare_parameter<int>("rng_seed", 12345);

  range_method_name_ = declare_parameter<std::string>("range_method", "cddt");
  theta_disc_        = declare_parameter<int>("theta_discretization", 112);
  max_range_m_       = declare_parameter<double>("max_range", 10.0);
  squash_factor_     = declare_parameter<double>("squash_factor", 2.2);
  inv_squash_        = 1.0 / squash_factor_;

  z_hit_       = declare_parameter<double>("z_hit", 0.85);
  z_short_     = declare_parameter<double>("z_short", 0.10);
  z_max_       = declare_parameter<double>("z_max", 0.025);
  z_rand_      = declare_parameter<double>("z_rand", 0.025);
  sigma_hit_   = declare_parameter<double>("sigma_hit", 0.1);
  lambda_short_= declare_parameter<double>("lambda_short", 0.25);

  lidar_aspect_ratio_ = declare_parameter<double>("lidar_aspect_ratio", 3.5);
  des_lidar_beams_    = declare_parameter<int>("des_lidar_beams", 21);

  laser_offset_x_ = declare_parameter<double>("laser_offset_x", 0.275);
  laser_offset_y_ = declare_parameter<double>("laser_offset_y", 0.0);

  resample_method_ = declare_parameter<std::string>("resample_method", "systematic");

  viz_              = declare_parameter<bool>("viz", false);
  publish_tf_       = declare_parameter<bool>("publish_tf", false);
  max_viz_particles_= declare_parameter<int>("max_viz_particles", 50);
  base_frame_       = declare_parameter<std::string>("base_frame", "base_link");
  odom_frame_       = declare_parameter<std::string>("odom_frame", "odom");

  rng_.seed(static_cast<uint32_t>(rng_seed_));
  particles_     = Eigen::MatrixXd::Zero(max_particles_, 3);
  proposal_buf_  = Eigen::MatrixXd::Zero(max_particles_, 3);
  weights_       = Eigen::VectorXd::Constant(max_particles_, 1.0 / max_particles_);
  queries_buf_.assign(static_cast<size_t>(max_particles_) * 3, 0.0f);
  sm_weights_.assign(static_cast<size_t>(max_particles_), 0.0);
  cdf_buf_.assign(static_cast<size_t>(max_particles_), 0.0);

  // TF
  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Latched map QoS (TRANSIENT_LOCAL incompatible with intra-process — disable IPC on this sub)
  rclcpp::QoS map_qos(1);
  map_qos.transient_local().reliable();
  rclcpp::SubscriptionOptions map_opts;
  map_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos, std::bind(&SynPFComponent::map_cb, this, _1), map_opts);
  // depth-1 keep-last so that if an update overruns the scan period, queued
  // stale scans are dropped and the next callback always gets the latest scan.
  sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&SynPFComponent::lidar_cb, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(5), std::bind(&SynPFComponent::odom_cb, this, _1));
  sub_initpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(1), std::bind(&SynPFComponent::clicked_pose_cb, this, _1));
  sub_clickpt_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", rclcpp::QoS(1), std::bind(&SynPFComponent::clicked_point_cb, this, _1));

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_pub_topic_, rclcpp::QoS(1));
  pub_particles_ = create_publisher<geometry_msgs::msg::PoseArray>("/pf/viz/particles", rclcpp::QoS(1));

  // In scan-triggered mode the MCL step runs from lidar_cb; otherwise a fixed-rate timer drives it.
  if (!trigger_on_scan_) {
    auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    main_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SynPFComponent::main_timer_cb, this));
  }

  RCLCPP_INFO(get_logger(),
    "syn_pf_cpp skeleton up. scan=%s odom=%s pose_pub=%s particles=%d %s",
    scan_topic_.c_str(), odom_topic_.c_str(), pose_pub_topic_.c_str(),
    max_particles_,
    trigger_on_scan_ ? "trigger=scan (event-driven)"
                     : ("trigger=timer @" + std::to_string(update_rate_hz_) + "Hz").c_str());
}

SynPFComponent::~SynPFComponent() = default;

void SynPFComponent::map_cb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (map_ready_.load()) return;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    last_map_ = msg;
    build_omap_from_msg(*msg);
    build_range_method();
  }
  map_ready_.store(true);
  RCLCPP_INFO(get_logger(),
    "Map: %ux%u, res=%.4f m, max_range_px=%d, range_method=%s",
    msg->info.width, msg->info.height, msg->info.resolution,
    max_range_px_, range_method_name_.c_str());
  try_init_sensor();
}

void SynPFComponent::lidar_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    last_scan_ = msg;
    if (!lidar_geom_ready_) {
      num_lidar_beams_ = static_cast<int>(msg->ranges.size());
      start_theta_ = msg->angle_min;
      end_theta_   = msg->angle_max;
      compute_boxed_indices();
      lidar_geom_ready_ = true;
      lidar_ready_.store(true);
      obs_buf_.assign(lidar_sample_idxs_.size(), 0.0f);
      RCLCPP_INFO(get_logger(),
        "Lidar geom: %d beams, fov=[%.2f, %.2f], sampled=%zu",
        num_lidar_beams_, start_theta_, end_theta_, lidar_sample_idxs_.size());
      try_init_sensor();
      return;
    }
    // Downsample ranges
    const float max_r = static_cast<float>(max_range_m_);
    downsampled_ranges_.resize(lidar_sample_idxs_.size());
    for (size_t k = 0; k < lidar_sample_idxs_.size(); ++k) {
      float r = msg->ranges[lidar_sample_idxs_[k]];
      if (!std::isfinite(r)) r = max_r;
      downsampled_ranges_[k] = r;
    }
  }  // release state_mtx_ before the (heavy) update; update_filter re-locks for the odom snapshot

  // Scan-triggered mode: run the full MCL step now, on this freshly-arrived scan.
  if (trigger_on_scan_) {
    update_filter();
  }
}

void SynPFComponent::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  last_odom_ = msg;
  if (!odom_ready_.load()) {
    odom_ready_.store(true);
    RCLCPP_INFO(get_logger(), "Odom in.");
  }
}

void SynPFComponent::clicked_pose_cb(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  double yaw = quat_to_yaw(q.x, q.y, q.z, q.w);
  initialize_particles_pose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
  have_last_pose_ = false;
  RCLCPP_INFO(get_logger(), "Init pose @ (%.2f, %.2f, %.1f deg).",
              msg->pose.pose.position.x, msg->pose.pose.position.y,
              yaw * 180.0 / M_PI);
}

void SynPFComponent::clicked_point_cb(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr /*msg*/)
{
  if (!map_ready_.load()) {
    RCLCPP_WARN(get_logger(), "Global re-init: waiting for map.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Global re-init (clicked_point).");
  initialize_global();
  have_last_pose_ = false;
}

void SynPFComponent::main_timer_cb()
{
  update_filter();
}

void SynPFComponent::update_filter()
{
  if (!map_ready_.load() || !lidar_ready_.load() || !odom_ready_.load()) {
    return;
  }
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    odom = last_odom_;
  }
  if (!odom) return;

  const auto & p = odom->pose.pose.position;
  const auto & q = odom->pose.pose.orientation;
  double yaw = quat_to_yaw(q.x, q.y, q.z, q.w);
  Eigen::Vector3d curr_pose(p.x, p.y, yaw);
  last_odom_vx_ = odom->twist.twist.linear.x;

  if (!particles_initialized_) {
    if (use_initial_pose_) {
      initialize_particles_pose(initial_x_, initial_y_, initial_theta_);
    } else if (map_ready_.load()) {
      initialize_global();  // Python parity: random over permissible cells
    } else {
      return;  // wait for map
    }
    last_pose_ = curr_pose;
    have_last_pose_ = true;
    return;
  }

  const auto t_start = std::chrono::steady_clock::now();

  if (!have_last_pose_) {
    have_last_pose_ = true;
    last_pose_ = curr_pose;
  } else {
    auto t0 = std::chrono::steady_clock::now();
    resample();
    auto t1 = std::chrono::steady_clock::now();
    // Only advance last_pose_ when motion_model_tum actually consumed the
    // delta (d_trans_raw >= its 1cm threshold) -- update_filter() runs on
    // every /scan message (~250Hz in sim), so at typical driving speeds the
    // inter-scan displacement is routinely sub-centimeter. Advancing
    // last_pose_ unconditionally every call (as before) meant the reference
    // point chased curr_pose every ~4ms and the accumulated displacement
    // never had a chance to cross the threshold -- particles never moved,
    // freezing the published pose the instant the car started driving.
    // Leaving last_pose_ untouched on a skip lets the sub-threshold gaps
    // accumulate until they're large enough for the motion model to use.
    const bool moved = motion_model_tum(curr_pose, last_pose_, last_odom_vx_);
    auto t2 = std::chrono::steady_clock::now();
    if (sensor_ready_.load() && !downsampled_ranges_.empty()) {
      sensor_update(weights_);
    }
    auto t3 = std::chrono::steady_clock::now();
    if (sensor_ready_.load() && !downsampled_ranges_.empty()) {
      permissible_clip(weights_);
      const double s = weights_.sum();
      if (s <= 0.0) {
        weights_.setConstant(1.0 / max_particles_);
      } else {
        weights_ /= s;
      }
    }
    auto t4 = std::chrono::steady_clock::now();

    using D = std::chrono::duration<double, std::milli>;
    push_stage(t_resample_,    D(t1 - t0).count());
    push_stage(t_motion_,      D(t2 - t1).count());
    push_stage(t_sensor_,      D(t3 - t2).count());
    push_stage(t_mcl_post_,    D(t4 - t3).count());
    if (moved) {
      last_pose_ = curr_pose;
    }
  }

  auto t_pe0 = std::chrono::steady_clock::now();
  Eigen::Vector3d mean = weighted_mean_pose();
  auto t_pe1 = std::chrono::steady_clock::now();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = "map";
  pose.pose.position.x = mean[0];
  pose.pose.position.y = mean[1];
  pose.pose.position.z = 0.0;
  double half = 0.5 * mean[2];
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(half);
  pose.pose.orientation.w = std::cos(half);
  pub_pose_->publish(pose);

  if (publish_tf_) {
    broadcast_map_to_odom(mean, pose.header.stamp);
  }
  if (viz_) {
    publish_particles_viz(pose.header.stamp);
  }

  auto t_end = std::chrono::steady_clock::now();
  using D = std::chrono::duration<double, std::milli>;
  push_stage(t_pose_extract_, D(t_pe1 - t_pe0).count());
  push_stage(t_publish_,      D(t_end - t_pe1).count());
  push_stage(t_total_,        D(t_end - t_start).count());
  maybe_report_stages();
}

// ---------- Helpers ----------
double SynPFComponent::angle_diff(double a, double b)
{
  double d = a - b;
  while (d >  M_PI) d -= 2.0 * M_PI;
  while (d < -M_PI) d += 2.0 * M_PI;
  return d;
}

double SynPFComponent::wrap_atan2(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double SynPFComponent::quat_to_yaw(double qx, double qy, double qz, double qw)
{
  return std::atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz));
}

void SynPFComponent::initialize_particles_pose(double x, double y, double theta)
{
  std::normal_distribution<double> nx(0.0, init_var_x_);
  std::normal_distribution<double> ny(0.0, init_var_y_);
  std::normal_distribution<double> nt(0.0, init_var_th_);
  for (int i = 0; i < max_particles_; ++i) {
    particles_(i, 0) = x + nx(rng_);
    particles_(i, 1) = y + ny(rng_);
    particles_(i, 2) = theta + nt(rng_);
  }
  weights_.setConstant(1.0 / max_particles_);
  particles_initialized_ = true;
}

bool SynPFComponent::motion_model_tum(
  const Eigen::Vector3d & curr, const Eigen::Vector3d & last, double odom_vx)
{
  const double dx = curr[0] - last[0];
  const double dy = curr[1] - last[1];
  double dtheta  = angle_diff(curr[2], last[2]);
  const double d_trans_raw = std::hypot(dx, dy);

  if (d_trans_raw < 0.01) return false;

  double d_rot1 = angle_diff(std::atan2(dy, dx), last[2]);

  double reverse_offset = 0.0;
  double reverse_spread = 1.0;
  if (odom_vx < -0.05) {
    reverse_offset = M_PI;
    reverse_spread = 1.1;
    d_rot1 += (d_rot1 < -M_PI / 2.0) ? M_PI : -M_PI;
  }

  double d_rot2 = angle_diff(dtheta, d_rot1);
  d_rot1 = std::min(std::abs(angle_diff(d_rot1, 0.0)),
                    std::abs(angle_diff(d_rot1, M_PI)));
  d_rot2 = std::min(std::abs(angle_diff(d_rot2, 0.0)),
                    std::abs(angle_diff(d_rot2, M_PI)));

  const double d_trans_safe = std::max(d_trans_raw, lam_thresh_);
  double scale_rot1  = alpha1_ * d_rot1 + alpha2_ / d_trans_safe;
  double scale_rot2  = alpha1_ * d_rot2 + alpha2_ / d_trans_safe;
  double scale_trans = alpha3_ * d_trans_raw + alpha4_ * (d_rot1 + d_rot2);

  scale_rot1  *= reverse_spread;
  scale_rot2  *= reverse_spread;
  scale_trans *= reverse_spread;

  std::normal_distribution<double> n_rot1(0.0, scale_rot1);
  std::normal_distribution<double> n_trans(scale_trans / 2.0, scale_trans);
  std::normal_distribution<double> n_rot2(0.0, scale_rot2);

  for (int i = 0; i < max_particles_; ++i) {
    const double dr1 = d_rot1  + n_rot1(rng_);
    const double dtr = d_trans_raw + n_trans(rng_);
    const double dr2 = d_rot2  + n_rot2(rng_);

    const double eff_hdg = particles_(i, 2) + dr1 + reverse_offset;
    particles_(i, 0) += dtr * std::cos(eff_hdg);
    particles_(i, 1) += dtr * std::sin(eff_hdg);
    particles_(i, 2) = wrap_atan2(particles_(i, 2) + dr1 + dr2);
  }
  return true;
}

Eigen::Vector3d SynPFComponent::weighted_mean_pose() const
{
  double mx = 0.0, my = 0.0, ms = 0.0, mc = 0.0;
  for (int i = 0; i < max_particles_; ++i) {
    const double w = weights_(i);
    mx += w * particles_(i, 0);
    my += w * particles_(i, 1);
    ms += w * std::sin(particles_(i, 2));
    mc += w * std::cos(particles_(i, 2));
  }
  return Eigen::Vector3d(mx, my, std::atan2(ms, mc));
}

// ---------- Map / rangelib init ----------
void SynPFComponent::build_omap_from_msg(const nav_msgs::msg::OccupancyGrid & msg)
{
  map_w_ = msg.info.width;
  map_h_ = msg.info.height;
  // Match Python (PyOMap from OccupancyGrid): OMap(height, width)
  omap_ = std::make_unique<ranges::OMap>(static_cast<int>(map_h_), static_cast<int>(map_w_));

  permissible_.assign(static_cast<size_t>(map_w_) * map_h_, 0);
  for (uint32_t y = 0; y < map_h_; ++y) {
    for (uint32_t x = 0; x < map_w_; ++x) {
      const auto v = msg.data[y * map_w_ + x];
      // Match Python: array_255[y,x] > 10 -> occupied
      if (v > 10) {
        omap_->grid[y][x] = true;
      }
      // Permissible: only cells == 0 (free)
      if (v == 0) {
        permissible_[y * map_w_ + x] = 1;
      }
    }
  }

  // World coord transform fields
  const auto & q = msg.info.origin.orientation;
  double yaw = quat_to_yaw(q.x, q.y, q.z, q.w);
  double angle = -1.0 * yaw;
  omap_->world_scale     = msg.info.resolution;
  omap_->world_angle     = static_cast<float>(angle);
  omap_->world_origin_x  = msg.info.origin.position.x;
  omap_->world_origin_y  = msg.info.origin.position.y;
  omap_->world_sin_angle = std::sin(angle);
  omap_->world_cos_angle = std::cos(angle);

  max_range_px_ = static_cast<int>(max_range_m_ / msg.info.resolution);
}

void SynPFComponent::build_range_method()
{
  if (!omap_) return;
  // Two-step eval (matches Python rangelib_variant=2): numpy_calc_range_angles ->
  // eval_sensor_model. Works for rmgpu (one-shot variant is unimplemented on GPU).
  auto make_eval = [this](auto* rm) {
    return [rm, this](float* ins, float* ang, float* obs, double* w, int n, int na) {
      const size_t need = static_cast<size_t>(n) * na;
      if (ranges_buf_.size() < need) ranges_buf_.resize(need);
      rm->numpy_calc_range_angles(ins, ang, ranges_buf_.data(), n, na);
      rm->eval_sensor_model(obs, ranges_buf_.data(), w, na, n);
    };
  };

  if (range_method_name_ == "bl") {
    auto* rm = new ranges::BresenhamsLine(*omap_, max_range_px_);
    range_method_.reset(rm);
    eval_fn_ = make_eval(rm);
    set_sensor_fn_ = [rm](double* t, int w) { rm->set_sensor_model(t, w); };
  } else if (range_method_name_ == "cddt" || range_method_name_ == "pcddt") {
    auto* rm = new ranges::CDDTCast(*omap_, max_range_px_, theta_disc_);
    if (range_method_name_ == "pcddt") {
      RCLCPP_INFO(get_logger(), "Pruning CDDT...");
      rm->prune(static_cast<float>(max_range_px_));
    }
    range_method_.reset(rm);
    eval_fn_ = make_eval(rm);
    set_sensor_fn_ = [rm](double* t, int w) { rm->set_sensor_model(t, w); };
  } else if (range_method_name_ == "rm") {
    auto* rm = new ranges::RayMarching(*omap_, max_range_px_);
    range_method_.reset(rm);
    eval_fn_ = make_eval(rm);
    set_sensor_fn_ = [rm](double* t, int w) { rm->set_sensor_model(t, w); };
#ifdef USE_CUDA
  } else if (range_method_name_ == "rmgpu") {
    auto* rm = new ranges::RayMarchingGPU(*omap_, max_range_px_);
    range_method_.reset(rm);
    eval_fn_ = make_eval(rm);
    set_sensor_fn_ = [rm](double* t, int w) { rm->set_sensor_model(t, w); };
#endif
  } else if (range_method_name_ == "glt") {
    auto* rm = new ranges::GiantLUTCast(*omap_, max_range_px_, theta_disc_);
    range_method_.reset(rm);
    eval_fn_ = make_eval(rm);
    set_sensor_fn_ = [rm](double* t, int w) { rm->set_sensor_model(t, w); };
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown range_method: %s", range_method_name_.c_str());
    throw std::runtime_error("Unknown range_method");
  }
}

void SynPFComponent::precompute_sensor_model()
{
  RCLCPP_INFO(get_logger(), "Precomputing sensor model table...");
  const double res = omap_->world_scale;
  const double sigma_hit_px = sigma_hit_ / res;
  const double lam_short_px = lambda_short_ / res;
  const double inv_max = 1.0 / static_cast<double>(max_range_px_);
  table_width_ = max_range_px_ + 1;
  sensor_model_table_.assign(table_width_, std::vector<double>(table_width_, 0.0));

  std::vector<double> norm_gau(table_width_, 0.0);
  std::vector<double> norm_exp(table_width_, 0.0);
  const double inv_two_sig2 = 1.0 / (2.0 * sigma_hit_px * sigma_hit_px);
  const double gau_denom = sigma_hit_px * std::sqrt(2.0 * M_PI);

  for (int d = 0; d < table_width_; ++d) {
    double sum_gau = 0.0, sum_exp = 0.0;
    for (int r = 0; r < table_width_; ++r) {
      const double z = static_cast<double>(d - r);
      sum_gau += std::exp(-z * z * inv_two_sig2) / gau_denom;
      if (r <= d) sum_exp += lam_short_px * std::exp(-lam_short_px * r);
    }
    norm_gau[d] = 1.0 / sum_gau;
    norm_exp[d] = sum_exp > 0.0 ? 1.0 / sum_exp : 1.0;
  }

  for (int d = 0; d < table_width_; ++d) {
    double col_sum = 0.0;
    for (int r = 0; r < table_width_; ++r) {
      const double z = static_cast<double>(d - r);
      double prob = z_hit_ * (std::exp(-z * z * inv_two_sig2) / gau_denom) * norm_gau[d];
      if (r <= d) prob += z_short_ * (lam_short_px * std::exp(-lam_short_px * r)) * norm_exp[d];
      if (r == max_range_px_) prob += z_max_;
      if (r < max_range_px_) prob += z_rand_ * inv_max;
      col_sum += prob;
      sensor_model_table_[r][d] = prob;
    }
    for (int r = 0; r < table_width_; ++r) {
      sensor_model_table_[r][d] /= col_sum;
    }
  }

  // Push table into rangelib (flat row-major double*)
  std::vector<double> flat(static_cast<size_t>(table_width_) * table_width_, 0.0);
  for (int i = 0; i < table_width_; ++i) {
    for (int j = 0; j < table_width_; ++j) {
      flat[i * table_width_ + j] = sensor_model_table_[i][j];
    }
  }
  set_sensor_fn_(flat.data(), table_width_);
}

void SynPFComponent::compute_boxed_indices()
{
  // Match Python get_boxed_indices(): pick DES_LIDAR_BEAMS sparse indices over a virtual box.
  std::vector<double> beam_angles(num_lidar_beams_);
  const double step = (end_theta_ - start_theta_) / std::max(num_lidar_beams_ - 1, 1);
  for (int i = 0; i < num_lidar_beams_; ++i) beam_angles[i] = start_theta_ + i * step;
  const int MID = num_lidar_beams_ / 2;
  std::vector<int> sparse_idxs{MID};

  const double a = lidar_aspect_ratio_;
  std::vector<std::pair<double, double>> beam_proj(num_lidar_beams_);
  for (int i = 0; i < num_lidar_beams_; ++i) {
    beam_proj[i] = {2.0 * a * std::cos(beam_angles[i]), 2.0 * a * std::sin(beam_angles[i])};
  }
  std::vector<std::pair<double, double>> beam_inter(num_lidar_beams_, {0.0, 0.0});

  std::vector<std::pair<double, double>> box_corners{
    {a, 1.0}, {a, -1.0}, {-a, -1.0}, {-a, 1.0}};
  for (int idx = 0; idx < 4; ++idx) {
    auto [x1, y1] = box_corners[idx];
    auto [x2, y2] = (idx == 3) ? box_corners[0] : box_corners[idx + 1];
    for (int i = 0; i < num_lidar_beams_; ++i) {
      const double x4 = beam_proj[i].first;
      const double y4 = beam_proj[i].second;
      const double den = (x1 - x2) * (-y4) - (y1 - y2) * (-x4);
      if (den == 0.0) continue;
      const double t = (x1 * (-y4) - y1 * (-x4)) / den;
      const double u = (x1 * (y1 - y2) - y1 * (x1 - x2)) / den;
      if (0.0 <= t && t <= 1.0 && 0.0 <= u && u <= 1.0) {
        beam_inter[i] = {u * x4, u * y4};
      }
    }
  }
  std::vector<double> dist(num_lidar_beams_ - 1, 0.0);
  double total = 0.0;
  for (int i = 0; i < num_lidar_beams_ - 1; ++i) {
    const double dx = beam_inter[i + 1].first - beam_inter[i].first;
    const double dy = beam_inter[i + 1].second - beam_inter[i].second;
    dist[i] = std::sqrt(dx * dx + dy * dy);
    total += dist[i];
  }
  const double dist_amt = total / static_cast<double>(des_lidar_beams_ - 1);

  int idx = MID + 1;
  const int DES2 = des_lidar_beams_ / 2 + 1;
  double acc = 0.0;
  while (static_cast<int>(sparse_idxs.size()) <= DES2) {
    if (idx >= static_cast<int>(dist.size())) break;
    acc += dist[idx];
    if (acc >= dist_amt) {
      acc = 0.0;
      sparse_idxs.push_back(idx - 1);
    }
    ++idx;
    if (idx == num_lidar_beams_ - 1) {
      sparse_idxs.push_back(num_lidar_beams_ - 1);
      break;
    }
  }
  std::vector<int> mirrored;
  for (size_t k = 1; k < sparse_idxs.size(); ++k) {
    mirrored.insert(mirrored.begin(), 2 * sparse_idxs[0] - sparse_idxs[k]);
  }
  std::vector<int> combined;
  combined.reserve(mirrored.size() + sparse_idxs.size());
  combined.insert(combined.end(), mirrored.begin(), mirrored.end());
  combined.insert(combined.end(), sparse_idxs.begin(), sparse_idxs.end());

  lidar_sample_idxs_ = std::move(combined);
  lidar_theta_lut_.resize(lidar_sample_idxs_.size());
  for (size_t k = 0; k < lidar_sample_idxs_.size(); ++k) {
    lidar_theta_lut_[k] = static_cast<float>(beam_angles[lidar_sample_idxs_[k]]);
  }
}

void SynPFComponent::try_init_sensor()
{
  if (sensor_ready_.load()) return;
  if (!map_ready_.load() || !lidar_geom_ready_) return;
  precompute_sensor_model();
  sensor_ready_.store(true);
  RCLCPP_INFO(get_logger(), "Sensor model ready. Particle filter running.");
}

void SynPFComponent::sensor_update(Eigen::VectorXd & weights)
{
  const int num_angles = static_cast<int>(lidar_theta_lut_.size());
  if (obs_buf_.size() != static_cast<size_t>(num_angles)) {
    obs_buf_.assign(num_angles, 0.0f);
  }
  for (int k = 0; k < num_angles; ++k) {
    obs_buf_[k] = downsampled_ranges_[k];
  }

  // Build queries: particles + R(theta) @ laser_offset
  const double lx = laser_offset_x_;
  const double ly = laser_offset_y_;
  for (int i = 0; i < max_particles_; ++i) {
    const double th = particles_(i, 2);
    const double c = std::cos(th);
    const double s = std::sin(th);
    queries_buf_[3 * i + 0] = static_cast<float>(particles_(i, 0) + c * lx - s * ly);
    queries_buf_[3 * i + 1] = static_cast<float>(particles_(i, 1) + s * lx + c * ly);
    queries_buf_[3 * i + 2] = static_cast<float>(th);
  }

  eval_fn_(
    queries_buf_.data(),
    lidar_theta_lut_.data(),
    obs_buf_.data(),
    sm_weights_.data(),
    max_particles_,
    num_angles);

  for (int i = 0; i < max_particles_; ++i) {
    weights(i) = std::pow(sm_weights_[i], inv_squash_);
  }
}

void SynPFComponent::resample()
{
  // Build CDF
  double acc = 0.0;
  for (int i = 0; i < max_particles_; ++i) {
    acc += weights_(i);
    cdf_buf_[i] = acc;
  }
  if (acc <= 0.0) {
    // No usable weights — keep particles, just reset weights uniform
    weights_.setConstant(1.0 / max_particles_);
    return;
  }
  const double inv_total = 1.0 / acc;

  if (resample_method_ == "multinomial") {
    // Match Python np.random.choice(particle_indices, N, p=weights)
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int i = 0; i < max_particles_; ++i) {
      const double r = u(rng_);
      int lo = 0, hi = max_particles_ - 1;
      while (lo < hi) {
        const int mid = (lo + hi) / 2;
        if (cdf_buf_[mid] * inv_total < r) lo = mid + 1;
        else hi = mid;
      }
      proposal_buf_.row(i) = particles_.row(lo);
    }
  } else {  // 'systematic' (default) — single stratified uniform draw
    std::uniform_real_distribution<double> u(0.0, 1.0 / max_particles_);
    const double start = u(rng_);
    int j = 0;
    for (int i = 0; i < max_particles_; ++i) {
      const double pos = start + static_cast<double>(i) / max_particles_;
      while (j < max_particles_ - 1 && cdf_buf_[j] * inv_total < pos) ++j;
      proposal_buf_.row(i) = particles_.row(j);
    }
  }
  particles_.swap(proposal_buf_);
}

void SynPFComponent::permissible_clip(Eigen::VectorXd & weights)
{
  if (permissible_.empty() || !omap_) return;
  const double inv_scale = 1.0 / omap_->world_scale;
  const double ox = omap_->world_origin_x;
  const double oy = omap_->world_origin_y;
  const double cw = omap_->world_cos_angle;
  const double sw = omap_->world_sin_angle;

  for (int i = 0; i < max_particles_; ++i) {
    // World->map (mirror PyOMap's world_to_map convention)
    double xw = particles_(i, 0) - ox;
    double yw = particles_(i, 1) - oy;
    double xr =  cw * xw + sw * yw;
    double yr = -sw * xw + cw * yw;
    int mx = static_cast<int>(xr * inv_scale);
    int my = static_cast<int>(yr * inv_scale);
    if (mx < 0) mx = 0;
    if (my < 0) my = 0;
    if (mx >= static_cast<int>(map_w_)) mx = map_w_ - 1;
    if (my >= static_cast<int>(map_h_)) my = map_h_ - 1;
    if (!permissible_[static_cast<size_t>(my) * map_w_ + mx]) {
      weights(i) *= 0.01;
    }
  }
}

void SynPFComponent::initialize_global()
{
  if (!omap_ || permissible_.empty()) {
    RCLCPP_WARN(get_logger(), "initialize_global: permissible region not ready");
    return;
  }
  // Collect free-cell indices
  std::vector<std::pair<int, int>> free_cells;
  free_cells.reserve(static_cast<size_t>(map_w_) * map_h_ / 4);
  for (uint32_t y = 0; y < map_h_; ++y) {
    for (uint32_t x = 0; x < map_w_; ++x) {
      if (permissible_[static_cast<size_t>(y) * map_w_ + x]) {
        free_cells.emplace_back(static_cast<int>(x), static_cast<int>(y));
      }
    }
  }
  if (free_cells.empty()) {
    RCLCPP_ERROR(get_logger(), "initialize_global: no free cells in map!");
    return;
  }
  RCLCPP_INFO(get_logger(),
    "initialize_global: scattering %d particles over %zu free cells.",
    max_particles_, free_cells.size());

  const double scale = omap_->world_scale;
  const double ox = omap_->world_origin_x;
  const double oy = omap_->world_origin_y;
  const double cw = omap_->world_cos_angle;
  const double sw = omap_->world_sin_angle;

  std::uniform_int_distribution<size_t> ucell(0, free_cells.size() - 1);
  std::uniform_real_distribution<double> utheta(0.0, 2.0 * M_PI);
  for (int i = 0; i < max_particles_; ++i) {
    auto [mx, my] = free_cells[ucell(rng_)];
    const double xr = mx * scale;
    const double yr = my * scale;
    // Inverse of world_to_map rotation: xw = cw*xr - sw*yr + ox
    particles_(i, 0) = cw * xr - sw * yr + ox;
    particles_(i, 1) = sw * xr + cw * yr + oy;
    particles_(i, 2) = utheta(rng_);
  }
  weights_.setConstant(1.0 / max_particles_);
  particles_initialized_ = true;
}

void SynPFComponent::publish_particles_viz(const rclcpp::Time & stamp)
{
  if (pub_particles_->get_subscription_count() == 0) return;
  geometry_msgs::msg::PoseArray pa;
  pa.header.stamp = stamp;
  pa.header.frame_id = "map";

  const int n_show = std::min(max_viz_particles_, max_particles_);
  pa.poses.resize(n_show);

  if (max_particles_ > max_viz_particles_) {
    // Resample by weights for viz (Python parity)
    std::vector<double> cdf(max_particles_);
    double acc = 0.0;
    for (int i = 0; i < max_particles_; ++i) { acc += weights_(i); cdf[i] = acc; }
    const double inv_total = (acc > 0.0) ? 1.0 / acc : 1.0;
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int k = 0; k < n_show; ++k) {
      const double r = u(rng_);
      int lo = 0, hi = max_particles_ - 1;
      while (lo < hi) {
        const int mid = (lo + hi) / 2;
        if (cdf[mid] * inv_total < r) lo = mid + 1; else hi = mid;
      }
      const double th = particles_(lo, 2);
      pa.poses[k].position.x = particles_(lo, 0);
      pa.poses[k].position.y = particles_(lo, 1);
      pa.poses[k].orientation.z = std::sin(0.5 * th);
      pa.poses[k].orientation.w = std::cos(0.5 * th);
    }
  } else {
    for (int k = 0; k < n_show; ++k) {
      const double th = particles_(k, 2);
      pa.poses[k].position.x = particles_(k, 0);
      pa.poses[k].position.y = particles_(k, 1);
      pa.poses[k].orientation.z = std::sin(0.5 * th);
      pa.poses[k].orientation.w = std::cos(0.5 * th);
    }
  }
  pub_particles_->publish(pa);
}

void SynPFComponent::broadcast_map_to_odom(
  const Eigen::Vector3d & mean, const rclcpp::Time & stamp)
{
  // map_T_odom = map_T_base * inv(odom_T_base)
  geometry_msgs::msg::TransformStamped odom_tf_msg;
  try {
    odom_tf_msg = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "%s->%s TF miss: %s", odom_frame_.c_str(), base_frame_.c_str(), ex.what());
    return;
  }

  tf2::Transform map_T_base;
  map_T_base.setOrigin(tf2::Vector3(mean[0], mean[1], 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, mean[2]);
  map_T_base.setRotation(q);

  tf2::Transform odom_T_base;
  tf2::fromMsg(odom_tf_msg.transform, odom_T_base);
  tf2::Transform map_T_odom = map_T_base * odom_T_base.inverse();

  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = "map";
  out.child_frame_id = odom_frame_;
  out.transform = tf2::toMsg(map_T_odom);
  tf_broadcaster_->sendTransform(out);
}

void SynPFComponent::push_stage(std::deque<double> & q, double ms)
{
  if (q.size() >= kStageBuf) q.pop_front();
  q.push_back(ms);
}

static inline double pctl(std::vector<double> & v, double p)
{
  if (v.empty()) return 0.0;
  const size_t idx = std::min(v.size() - 1,
    static_cast<size_t>(p / 100.0 * (v.size() - 1)));
  std::nth_element(v.begin(), v.begin() + idx, v.end());
  return v[idx];
}

void SynPFComponent::maybe_report_stages()
{
  const auto now_t = std::chrono::steady_clock::now();
  if (last_report_t_.time_since_epoch().count() == 0) {
    last_report_t_ = now_t;
    return;
  }
  if (std::chrono::duration<double>(now_t - last_report_t_).count() < 10.0) return;
  last_report_t_ = now_t;

  auto stats = [](const std::deque<double> & q, const char * name) {
    if (q.size() < 10) return std::string{};
    std::vector<double> v(q.begin(), q.end());
    const double p50 = pctl(v, 50);
    const double p95 = pctl(v, 95);
    const double p99 = pctl(v, 99);
    char buf[128];
    snprintf(buf, sizeof(buf),
      "%s: p50=%.2f p95=%.2f p99=%.2fms (n=%zu)", name, p50, p95, p99, q.size());
    return std::string(buf);
  };

  std::string s = "C++ PF stage latency:\n  ";
  s += stats(t_resample_,     "resample");      s += "\n  ";
  s += stats(t_motion_,       "motion");        s += "\n  ";
  s += stats(t_sensor_,       "sensor");        s += "\n  ";
  s += stats(t_mcl_post_,     "mcl_post");      s += "\n  ";
  s += stats(t_pose_extract_, "pose_extract");  s += "\n  ";
  s += stats(t_publish_,      "publish");       s += "\n  ";
  s += stats(t_total_,        "update_total");
  RCLCPP_INFO(get_logger(), "%s", s.c_str());
}

}  // namespace syn_pf_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(syn_pf_cpp::SynPFComponent)
