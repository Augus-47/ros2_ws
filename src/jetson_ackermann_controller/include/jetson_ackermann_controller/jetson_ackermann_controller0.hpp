#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace jetson_ackermann_controller
{

class JetsonAckermannController : public nav2_core::Controller
{
public:
  JetsonAckermannController() = default;
  ~JetsonAckermannController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ---- core helpers ----
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  double max_steer_rate_deg_per_s_{50.0};

  bool enable_path_resample_{true};
  double path_resample_ds_{0.08};
 
  double w_lat_error_{1.0};
  double w_yaw_error_{0.4};

  double direction_switch_x_{0.25};

  bool enable_adaptive_sampling_span_{true};
  double sampling_span_min_ratio_{0.25};
  double sampling_span_max_ratio_{1.0};
  double sampling_span_obstacle_gain_{0.8};

  bool enable_obstacle_speed_scaling_{true};
  double obstacle_speed_cost_gain_{2.0};
  double obstacle_speed_min_scale_{0.2};


  bool transformPose(
    const std::string & frame,
    const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out) const;

  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const;

  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double lookahead_dist,
    const nav_msgs::msg::Path & transformed_plan) const;

  bool inCollision(const double x, const double y, const double theta) const;
  double costAtPose(const double x, const double y) const;
  double getCostmapMaxExtent() const;

  // ✅ 你原本就有的：接近终点减速上限（保留）
  double computeApproachSpeedLimit(double dist_to_goal) const;  // :contentReference[oaicite:1]{index=1}

  // 【优化点3】阿克曼曲率限速（大曲率自动降速，明显减少抖/蛇形）
  // v <= sqrt(max_lat_accel / |kappa|)
  double computeCurvatureSpeedLimit(double kappa_abs) const;

  // ---- avoidance sampling ----
  struct Candidate
  {
    double kappa{0.0};     // curvature (1/m)
    double v{0.0};         // linear velocity (m/s)
    double score{1e9};     // smaller is better
    bool valid{false};
  };

  // 【优化点4】evaluateCandidate 增加 carrot 全局坐标（进度项）+ 保留你原来的 transformed_plan_base 参数
  Candidate evaluateCandidate(
    const geometry_msgs::msg::PoseStamped & robot_pose_global,
    const nav_msgs::msg::Path & transformed_plan_base,  // 保留（你原来签名里有它 :contentReference[oaicite:2]{index=2}）
    double kappa_nom,
    double kappa_test,
    double v_sign,
    double v_des_max,                     // 本周期速度上限（接近终点减速后）
    const geometry_msgs::msg::Twist & curr_speed,
    double carrot_gx,                     // 【优化点4】carrot 全局坐标 x
    double carrot_gy) const;              // 【优化点4】carrot 全局坐标 y

  // lifecycle / context
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("JetsonAckermannController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>
    collision_checker_;

  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;

  // params
  double wheelbase_{0.8};
  double max_steer_deg_{45.0};

  double desired_speed_{0.5};
  double base_desired_speed_{0.5};
  double min_speed_{0.05};
  double max_speed_{0.5};   // 你说最高0.5足够：默认改成0.5
  double max_accel_{0.8};
  double max_decel_{1.2};

  bool allow_reversing_{true};
  double reverse_speed_{0.2};

  double lookahead_dist_{0.8};
  bool use_velocity_scaled_lookahead_{false};
  double lookahead_time_{1.2};
  double min_lookahead_{0.5};
  double max_lookahead_{1.5};

  // ✅ 平缓停车参数
  double min_approach_linear_velocity_{0.06};
  double approach_velocity_scaling_dist_{1.0};
  double stop_distance_{0.05};

  // sampling + scoring
  int num_kappa_samples_{11};     // odd number
  double horizon_time_{1.0};      // seconds
  double sim_dt_{0.05};           // seconds

  double w_cost_{1.0};
  double w_kappa_{0.8};
  double w_steer_change_{0.2};
  double w_reverse_{2.0};

  // 【优化点2】采样跨度比例：默认不要每次扫满全舵角（减少离散跳变引起的蛇形）
  double kappa_sample_span_ratio_{0.5}; // 0.2~0.8 常用

  // 【优化点3】曲率限速用：最大横向加速度（m/s^2）
  double max_lat_accel_{1.2};

  // 【优化点4】进度项权重：越大越不容易“最安全=停住”
  double w_progress_{0.6};

  // 【优化点5】高代价阈值：footprint cost >= 该值直接判无效，减少贴边蹭墙
  double forbidden_cost_threshold_{220.0}; // 0~253；设为253/254可近似“关闭”

  // 【优化点6】连续无解 N 次才触发倒车（避免偶发误触发）
  int no_valid_cmd_before_reverse_{5};
  int no_valid_cmd_count_{0};

  // derived
  double control_duration_{0.05}; // = 1/controller_frequency

  // ---- anti-oscillation state ----
  double prev_kappa_{0.0};
  bool has_prev_kappa_{false};
  double prev_v_sign_{1.0};       // +1 forward, -1 reverse

  // ---- reverse hold state ----
  double reverse_hold_time_{0.8};
  bool reverse_active_{false};
  rclcpp::Time reverse_until_{0, 0, RCL_ROS_TIME};

  // state
  mutable std::mutex mutex_;
};

}  // namespace jetson_ackermann_controller

