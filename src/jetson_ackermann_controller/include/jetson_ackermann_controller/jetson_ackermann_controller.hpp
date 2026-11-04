#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
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
  // ==============================
  // 路径 / TF / costmap 辅助函数
  // ==============================
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  void bevCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool bevAvailable() const;

  double legacyFootprintCostAtPose(double x, double y, double theta) const;
  double bevFootprintCostAtPose(double x, double y, double theta) const;

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

  double computeApproachSpeedLimit(double dist_to_goal) const;
  double computeCurvatureSpeedLimit(double kappa_abs) const;

  // ==============================
  // 候选轨迹
  // ==============================
  struct Candidate
  {
    double kappa{0.0};
    double v{0.0};
    double score{1e9};
    bool valid{false};
  };

  Candidate evaluateCandidate(
    const geometry_msgs::msg::PoseStamped & robot_pose_global,
    const nav_msgs::msg::Path & transformed_plan_base,
    double kappa_nom,
    double kappa_test,
    double v_sign,
    double v_des_max,
    const geometry_msgs::msg::Twist & curr_speed,
    double carrot_gx,
    double carrot_gy) const;

  // ==============================
  // 终端位姿微调：直接优化 final goal 的 x / y / yaw
  // ==============================
  Candidate evaluateTerminalCandidate(
    const geometry_msgs::msg::PoseStamped & robot_pose_global,
    double goal_x_base,
    double goal_y_base,
    double goal_yaw_base,
    double kappa_test,
    double v_sign,
    double v_des,
    const geometry_msgs::msg::Twist & curr_speed) const;

  geometry_msgs::msg::TwistStamped computeTerminalAlignCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    double goal_x_base,
    double goal_y_base,
    double goal_yaw_base);

  // ==============================
  // 生命周期 / 上下文
  // ==============================
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("JetsonAckermannController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  std::unique_ptr<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;

  // ==============================
  // BEV 局部 cost grid
  // ==============================
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr bev_sub_;

  std::unique_ptr<nav2_costmap_2d::Costmap2D> bev_costmap_;
  std::unique_ptr<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> bev_collision_checker_;

  mutable std::mutex bev_mutex_;

  bool bev_ready_{false};
  rclcpp::Time bev_stamp_{0, 0, RCL_ROS_TIME};

  std::string bev_topic_;
  double bev_timeout_{0.2};
  double bev_unknown_cost_{80.0};
  bool use_bev_as_primary_{true};
  bool bev_fallback_to_costmap_{true};

  // ==============================
  // TF 容错
  // ==============================
  double transform_tolerance_{0.2};
  bool allow_tf_latest_fallback_{true};

  // ==============================
  // 车辆与速度约束
  // ==============================
  double wheelbase_{0.8};

  // 注意：这里默认值改成和底盘一致。
  // 你的底盘发送层限制是 -31.86 ~ +31.86 deg。
  double max_steer_deg_{40.00};

  double max_steer_rate_deg_per_s_{80.0};

  double desired_speed_{0.5};
  double base_desired_speed_{0.5};
  double min_speed_{0.05};
  double max_speed_{0.5};
  double max_accel_{0.8};
  double max_decel_{1.2};

  bool allow_reversing_{true};
  double reverse_speed_{0.05};
  double direction_switch_x_{0.25};

  double lookahead_dist_{0.8};
  bool use_velocity_scaled_lookahead_{false};
  double lookahead_time_{1.2};
  double min_lookahead_{0.5};
  double max_lookahead_{1.5};

  bool enable_path_resample_{true};
  double path_resample_ds_{0.08};

  double min_approach_linear_velocity_{0.06};
  double approach_velocity_scaling_dist_{1.0};
  double stop_distance_{0.05};

  // ==============================
  // 普通路径跟踪采样 / 评分
  // ==============================
  int num_kappa_samples_{11};
  double horizon_time_{1.0};
  double sim_dt_{0.05};

  double w_cost_{1.0};
  double w_kappa_{0.8};
  double w_steer_change_{0.2};
  double w_reverse_{2.0};

  // ==============================
  // 【横向误差优化】
  // 这里默认值比原来大，因为现在要让 base_link 更严格贴路径。
  // w_lat_error_ 和 w_yaw_error_ 会作用在平方误差上。
  // ==============================
  double w_lat_error_{12.0};
  double w_yaw_error_{2.0};

  // ==============================
  // 【横向误差优化】
  // path_lat_kp_：横向误差反馈增益，越大越贴路径，但太大会左右摆。
  // path_yaw_kp_：路径航向误差反馈增益。
  // path_lat_deadband_：很小的横向误差死区，避免毫米级噪声导致抖动。
  // ==============================
  double path_lat_kp_{1.5};
  double path_yaw_kp_{0.8};
  double path_lat_deadband_{0.01};

  double kappa_sample_span_ratio_{0.5};
  double max_lat_accel_{1.2};
  double w_progress_{0.6};

  double forbidden_cost_threshold_{220.0};

  int no_valid_cmd_before_reverse_{5};
  int no_valid_cmd_count_{0};

  bool enable_adaptive_sampling_span_{true};
  double sampling_span_min_ratio_{0.25};
  double sampling_span_max_ratio_{1.0};
  double sampling_span_obstacle_gain_{0.8};

  bool enable_obstacle_speed_scaling_{true};
  double obstacle_speed_cost_gain_{2.0};
  double obstacle_speed_min_scale_{0.2};

  // ==============================
  // 终端位姿微调参数
  // 目标：xy <= 0.066m，yaw <= 17.19deg 左右
  // ==============================
  bool enable_terminal_align_{true};
  double terminal_align_dist_{0.3};
  double terminal_xy_tolerance_{0.066};
  double terminal_yaw_tolerance_{0.3};
  double terminal_max_speed_{0.06};
  double terminal_min_speed_{0.02};
  double terminal_horizon_time_{2.0};
  int terminal_num_kappa_samples_{21};

  double terminal_w_lon_error_{2.0};
  double terminal_w_lat_error_{10.0};
  double terminal_w_yaw_error_{6.0};
  double terminal_w_steer_change_{3.0};
  double terminal_w_reverse_{0.3};
  double terminal_w_direction_switch_{1.0};

  int terminal_stable_count_{0};
  int terminal_required_stable_count_{3};

  // ==============================
  // 终端保护：避免在目标附近反复磨、越修越偏
  // ==============================
  double terminal_timeout_{8.0};
  int terminal_no_improve_cycles_{160};
  double terminal_min_score_improve_{0.0001};

  double terminal_freeze_dist_{0.16};
  double terminal_leave_goal_margin_{0.04};
  double terminal_max_kappa_ratio_{0.55};

  bool terminal_allow_reverse_{true};
  double terminal_reverse_min_x_{0.05};

  // ==============================
  // 派生参数
  // ==============================
  double control_duration_{0.05};

  // ==============================
  // 历史状态
  // ==============================
  double prev_kappa_{0.0};
  bool has_prev_kappa_{false};
  double prev_v_sign_{1.0};

  // ==============================
  // 普通模式倒车恢复状态
  // ==============================
  double reverse_hold_time_{0.8};
  bool reverse_active_{false};
  rclcpp::Time reverse_until_{0, 0, RCL_ROS_TIME};

  // ==============================
  // 终端模式状态
  // ==============================
  bool terminal_active_{false};
  bool terminal_abort_{false};
  rclcpp::Time terminal_enter_time_{0, 0, RCL_ROS_TIME};
  double terminal_last_best_score_{1e9};
  int terminal_no_improve_count_{0};

  mutable std::mutex mutex_;
};

}  // namespace jetson_ackermann_controller

