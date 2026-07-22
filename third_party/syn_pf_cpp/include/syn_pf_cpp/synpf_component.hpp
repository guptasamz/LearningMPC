#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <functional>
#include <random>
#include <string>

#include <Eigen/Dense>

// rangelib (header-only). Compile-time flags from RangeLib.h: ROS_WORLD_TO_GRID=1, SENSOR_MODEL_HELPERS=1.
#include "RangeLib.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace syn_pf_cpp {

class SynPFComponent : public rclcpp::Node {
public:
  explicit SynPFComponent(const rclcpp::NodeOptions & options);
  ~SynPFComponent() override;

private:
  // ---------- Callbacks ----------
  void map_cb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void lidar_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void clicked_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void clicked_point_cb(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void main_timer_cb();
  void update_filter();   // full MCL step; called by timer or by lidar_cb (scan-triggered)

  // ---------- Subs / pubs ----------
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initpose_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clickpt_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---------- State flags ----------
  std::atomic<bool> map_ready_{false};
  std::atomic<bool> lidar_ready_{false};
  std::atomic<bool> odom_ready_{false};

  // ---------- Latest cached msgs ----------
  std::mutex state_mtx_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_scan_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_map_;

  // ---------- Params ----------
  std::string pose_pub_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  int max_particles_{3000};
  double update_rate_hz_{50.0};
  bool trigger_on_scan_{false};   // true: run MCL on each new scan instead of a fixed timer

  double init_var_x_{0.5}, init_var_y_{0.5}, init_var_th_{0.4};
  double alpha1_{0.5}, alpha2_{0.015}, alpha3_{1.0}, alpha4_{0.1};
  double lam_thresh_{0.1};
  bool use_initial_pose_{false};
  double initial_x_{0.0}, initial_y_{0.0}, initial_theta_{0.0};
  int rng_seed_{12345};

  // Viz / TF
  bool viz_{false};
  bool publish_tf_{false};
  int max_viz_particles_{50};
  std::string base_frame_{"base_link"};
  std::string odom_frame_{"odom"};

  // ---------- Particle state ----------
  Eigen::MatrixXd particles_;   // (N, 3): [x, y, theta]
  Eigen::VectorXd weights_;     // (N,)
  bool particles_initialized_{false};
  Eigen::Vector3d last_pose_{Eigen::Vector3d::Zero()};
  bool have_last_pose_{false};
  double last_odom_vx_{0.0};

  std::mt19937 rng_;

  // ---------- Sensor / map ----------
  std::unique_ptr<ranges::OMap> omap_;
  std::unique_ptr<ranges::RangeMethod> range_method_;
  // Type-erased dispatch (the one-shot eval is NOT virtual in rangelib base class;
  // calling via base ptr falls back to scalar calc_range which RayMarchingGPU rejects).
  std::function<void(float*, float*, float*, double*, int, int)> eval_fn_;
  std::function<void(double*, int)> set_sensor_fn_;
  std::vector<std::vector<double>> sensor_model_table_; // (table_width, table_width)
  int table_width_{0};
  int max_range_px_{0};
  double max_range_m_{10.0};
  std::string range_method_name_{"cddt"};
  int theta_disc_{112};

  // sensor model params
  double z_hit_{0.85}, z_short_{0.10}, z_max_{0.025}, z_rand_{0.025};
  double sigma_hit_{0.1}, lambda_short_{0.25};
  double squash_factor_{2.2};
  double inv_squash_{1.0 / 2.2};

  // permissible region (1=free, 0=occupied/unknown). Row-major (height, width)
  std::vector<uint8_t> permissible_;
  uint32_t map_w_{0}, map_h_{0};

  // lidar geometry
  int num_lidar_beams_{0};
  float start_theta_{0.0f}, end_theta_{0.0f};
  double lidar_aspect_ratio_{3.5};
  int des_lidar_beams_{21};
  std::vector<int> lidar_sample_idxs_;
  std::vector<float> lidar_theta_lut_;
  std::vector<float> downsampled_ranges_;
  bool lidar_geom_ready_{false};
  std::atomic<bool> sensor_ready_{false};

  // laser→base offset (m)
  double laser_offset_x_{0.275};
  double laser_offset_y_{0.0};

  // scratch buffers (reuse across ticks)
  std::vector<float> queries_buf_;   // (N, 3) row-major
  std::vector<double> sm_weights_;   // (N,) sensor model output
  std::vector<float> obs_buf_;       // (num_angles,)
  std::vector<float> ranges_buf_;    // (N*num_angles,) scratch for two-step eval

  // resample
  std::string resample_method_{"systematic"};  // 'systematic' | 'multinomial'
  Eigen::MatrixXd proposal_buf_;
  std::vector<double> cdf_buf_;

  // ---------- Stage timing (mirror Python instrumentation) ----------
  static constexpr size_t kStageBuf = 500;
  std::deque<double> t_resample_, t_motion_, t_sensor_, t_mcl_post_,
                     t_pose_extract_, t_publish_, t_total_;
  std::chrono::steady_clock::time_point last_report_t_;
  void push_stage(std::deque<double> & q, double ms);
  void maybe_report_stages();

  // ---------- Helpers ----------
  void initialize_particles_pose(double x, double y, double theta);
  void initialize_global();
  void publish_particles_viz(const rclcpp::Time & stamp);
  void broadcast_map_to_odom(const Eigen::Vector3d & mean, const rclcpp::Time & stamp);
  void motion_model_tum(
    const Eigen::Vector3d & curr, const Eigen::Vector3d & last, double odom_vx);
  void try_init_sensor();
  void build_omap_from_msg(const nav_msgs::msg::OccupancyGrid & msg);
  void build_range_method();
  void precompute_sensor_model();
  void compute_boxed_indices();
  void sensor_update(Eigen::VectorXd & weights);
  void permissible_clip(Eigen::VectorXd & weights);
  void resample();
  Eigen::Vector3d weighted_mean_pose() const;
  static double angle_diff(double a, double b);
  static double wrap_atan2(double a);
  static double quat_to_yaw(double qx, double qy, double qz, double qw);
};

}  // namespace syn_pf_cpp
