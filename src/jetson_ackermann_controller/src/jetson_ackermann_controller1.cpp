#include "jetson_ackermann_controller/jetson_ackermann_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace jetson_ackermann_controller
{

static double deg2rad(double d) { return d * M_PI / 180.0; }

void JetsonAckermannController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("JetsonAckermannController: unable to lock node");
  }

  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double controller_frequency = 20.0;
  declare_parameter_if_not_declared(node, "controller_frequency", rclcpp::ParameterValue(20.0));
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / std::max(1.0, controller_frequency);

  // ---- declare params ----
  declare_parameter_if_not_declared(node, plugin_name_ + ".reverse_hold_time", rclcpp::ParameterValue(0.8));

  declare_parameter_if_not_declared(node, plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_steer_deg", rclcpp::ParameterValue(45.0));

  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_speed", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.5));  // 你说最高0.5足够
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_accel", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel", rclcpp::ParameterValue(1.2));

  declare_parameter_if_not_declared(node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reverse_speed", rclcpp::ParameterValue(0.2));

  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_velocity_scaled_lookahead", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead", rclcpp::ParameterValue(1.5));

  declare_parameter_if_not_declared(node, plugin_name_ + ".num_kappa_samples", rclcpp::ParameterValue(11));
  declare_parameter_if_not_declared(node, plugin_name_ + ".horizon_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sim_dt", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_cost", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_kappa", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_steer_change", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_reverse", rclcpp::ParameterValue(2.0));

  // ✅ 平缓停车参数（保留）
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.06));
  declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".stop_distance", rclcpp::ParameterValue(0.05));

  // 【优化点2】采样跨度比例
  declare_parameter_if_not_declared(node, plugin_name_ + ".kappa_sample_span_ratio", rclcpp::ParameterValue(0.5));
  // 【优化点3】曲率限速
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lat_accel", rclcpp::ParameterValue(1.2));
  // 【优化点4】进度项
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_progress", rclcpp::ParameterValue(0.6));
  // 【优化点5】高代价阈值
  declare_parameter_if_not_declared(node, plugin_name_ + ".forbidden_cost_threshold", rclcpp::ParameterValue(220.0));
  // 【优化点6】连续无解N次才倒车
  declare_parameter_if_not_declared(node, plugin_name_ + ".no_valid_cmd_before_reverse", rclcpp::ParameterValue(5));

  // ---- get params ----
  node->get_parameter(plugin_name_ + ".reverse_hold_time", reverse_hold_time_);

  node->get_parameter(plugin_name_ + ".wheelbase", wheelbase_);
  node->get_parameter(plugin_name_ + ".max_steer_deg", max_steer_deg_);

  node->get_parameter(plugin_name_ + ".desired_speed", desired_speed_);
  base_desired_speed_ = desired_speed_;

  node->get_parameter(plugin_name_ + ".min_speed", min_speed_);
  node->get_parameter(plugin_name_ + ".max_speed", max_speed_);
  node->get_parameter(plugin_name_ + ".max_accel", max_accel_);
  node->get_parameter(plugin_name_ + ".max_decel", max_decel_);

  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter(plugin_name_ + ".reverse_speed", reverse_speed_);

  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead", use_velocity_scaled_lookahead_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".min_lookahead", min_lookahead_);
  node->get_parameter(plugin_name_ + ".max_lookahead", max_lookahead_);

  node->get_parameter(plugin_name_ + ".num_kappa_samples", num_kappa_samples_);
  num_kappa_samples_ = std::max(3, num_kappa_samples_);
  if (num_kappa_samples_ % 2 == 0) { num_kappa_samples_ += 1; }  // keep odd

  node->get_parameter(plugin_name_ + ".horizon_time", horizon_time_);
  node->get_parameter(plugin_name_ + ".sim_dt", sim_dt_);
  node->get_parameter(plugin_name_ + ".w_cost", w_cost_);
  node->get_parameter(plugin_name_ + ".w_kappa", w_kappa_);
  node->get_parameter(plugin_name_ + ".w_steer_change", w_steer_change_);
  node->get_parameter(plugin_name_ + ".w_reverse", w_reverse_);

  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".stop_distance", stop_distance_);

  // 【优化点2/3/4/5/6】新增参数读取
  node->get_parameter(plugin_name_ + ".kappa_sample_span_ratio", kappa_sample_span_ratio_);
  node->get_parameter(plugin_name_ + ".max_lat_accel", max_lat_accel_);
  node->get_parameter(plugin_name_ + ".w_progress", w_progress_);
  node->get_parameter(plugin_name_ + ".forbidden_cost_threshold", forbidden_cost_threshold_);
  node->get_parameter(plugin_name_ + ".no_valid_cmd_before_reverse", no_valid_cmd_before_reverse_);

  // 参数保护
  kappa_sample_span_ratio_ = std::clamp(kappa_sample_span_ratio_, 0.05, 1.0);
  max_lat_accel_ = std::max(0.0, max_lat_accel_);
  forbidden_cost_threshold_ = std::clamp(forbidden_cost_threshold_, 0.0, 253.0);
  no_valid_cmd_before_reverse_ = std::max(1, no_valid_cmd_before_reverse_);

  collision_checker_ =
    std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
  collision_checker_->setCostmap(costmap_);

  // reset states
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;

  RCLCPP_INFO(
    logger_,
    "JetsonAckermannController configured. wheelbase=%.3f max_steer=%.1fdeg desired=%.2f max_v=%.2f "
    "span_ratio=%.2f a_lat=%.2f w_prog=%.2f forbid=%.1f no_valid_before_rev=%d",
    wheelbase_, max_steer_deg_, desired_speed_, max_speed_,
    kappa_sample_span_ratio_, max_lat_accel_, w_progress_, forbidden_cost_threshold_,
    no_valid_cmd_before_reverse_);
}

void JetsonAckermannController::cleanup()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  no_valid_cmd_count_ = 0;  // 【优化点6】清零无解计数
}

void JetsonAckermannController::activate()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;  // 【优化点6】清零无解计数
}

void JetsonAckermannController::deactivate()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  no_valid_cmd_count_ = 0;  // 【优化点6】清零无解计数
}

void JetsonAckermannController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;

  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;  // 【优化点6】清零无解计数
}

void JetsonAckermannController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit < 0.0) {
    desired_speed_ = base_desired_speed_;
    return;
  }
  if (percentage) {
    desired_speed_ = base_desired_speed_ * speed_limit / 100.0;
  } else {
    desired_speed_ = speed_limit;
  }
  desired_speed_ = std::clamp(desired_speed_, 0.0, max_speed_);
}

double JetsonAckermannController::getCostmapMaxExtent() const
{
  const double max_dim = std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_dim / 2.0;
}

bool JetsonAckermannController::transformPose(
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id == frame) {
    out = in;
    return true;
  }
  try {
    tf_->transform(in, out, frame);
    out.header.frame_id = frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "transformPose failed: %s", ex.what());
    return false;
  }
}

nav_msgs::msg::Path JetsonAckermannController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("JetsonAckermannController: plan is empty");
  }

  geometry_msgs::msg::PoseStamped robot_pose_in_plan;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose_in_plan)) {
    throw nav2_core::PlannerException("JetsonAckermannController: can't transform robot pose to plan frame");
  }

  const double max_extent = getCostmapMaxExtent();

  auto closest_upper =
    nav2_util::geometry_utils::first_after_integrated_distance(
      global_plan_.poses.begin(), global_plan_.poses.end(), max_extent);

  auto begin_it = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_upper,
    [&](const auto & ps) {
      return nav2_util::geometry_utils::euclidean_distance(robot_pose_in_plan, ps);
    });

  auto end_it = std::find_if(
    begin_it, global_plan_.poses.end(),
    [&](const auto & ps) {
      return nav2_util::geometry_utils::euclidean_distance(ps, robot_pose_in_plan) > max_extent;
    });

  nav_msgs::msg::Path transformed;
  transformed.header.frame_id = costmap_ros_->getBaseFrameID(); // base_link
  transformed.header.stamp = pose.header.stamp;

  for (auto it = begin_it; it != end_it; ++it) {
    geometry_msgs::msg::PoseStamped p_in = *it;
    p_in.header.stamp = pose.header.stamp;
    geometry_msgs::msg::PoseStamped p_out;
    if (transformPose(costmap_ros_->getBaseFrameID(), p_in, p_out)) {
      p_out.pose.position.z = 0.0;
      transformed.poses.push_back(p_out);
    }
  }

  // prune
  global_plan_.poses.erase(global_plan_.poses.begin(), begin_it);

  if (transformed.poses.empty()) {
    throw nav2_core::PlannerException("JetsonAckermannController: transformed plan empty");
  }
  return transformed;
}

double JetsonAckermannController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const
{
  double d = lookahead_dist_;
  if (use_velocity_scaled_lookahead_) {
    d = std::fabs(speed.linear.x) * lookahead_time_;
    d = std::clamp(d, min_lookahead_, max_lookahead_);
  }
  return d;
}

geometry_msgs::msg::PoseStamped JetsonAckermannController::getLookAheadPoint(
  const double lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan) const
{
  auto it = std::find_if(transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
    return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
  });
  if (it == transformed_plan.poses.end()) {
    return transformed_plan.poses.back();
  }
  return *it;
}

bool JetsonAckermannController::inCollision(const double x, const double y, const double theta) const
{
  double cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());

  if (cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    return !costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  }

  // ✅ 只把 lethal 当碰撞（膨胀层交给 score 的 w_cost 处理）
  return cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

double JetsonAckermannController::costAtPose(const double x, const double y) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  return static_cast<double>(costmap_->getCost(mx, my));
}

// ✅ 你原有逻辑：接近终点速度上限（保留）
double JetsonAckermannController::computeApproachSpeedLimit(double dist_to_goal) const
{
  double v = std::clamp(desired_speed_, 0.0, max_speed_);

  if (stop_distance_ > 0.0 && dist_to_goal <= stop_distance_) {
    return 0.0;
  }

  if (approach_velocity_scaling_dist_ > 1e-6 && dist_to_goal < approach_velocity_scaling_dist_) {
    const double scale = std::clamp(dist_to_goal / approach_velocity_scaling_dist_, 0.0, 1.0);
    const double v_scaled = v * scale;
    v = std::max(min_approach_linear_velocity_, v_scaled);
  }

  return std::clamp(v, 0.0, max_speed_);
}

// 【优化点3】曲率限速：v <= sqrt(a_lat_max / |kappa|)
double JetsonAckermannController::computeCurvatureSpeedLimit(double kappa_abs) const
{
  if (max_lat_accel_ <= 1e-6) {
    return max_speed_;
  }
  const double eps = 1e-4;
  kappa_abs = std::max(kappa_abs, eps);
  const double v_lim = std::sqrt(std::max(0.0, max_lat_accel_) / kappa_abs);
  return std::clamp(v_lim, 0.0, max_speed_);
}

JetsonAckermannController::Candidate JetsonAckermannController::evaluateCandidate(
  const geometry_msgs::msg::PoseStamped & robot_pose_global,
  const nav_msgs::msg::Path & /*transformed_plan_base*/,
  double kappa_nom,
  double kappa_test,
  double v_sign,
  double v_des_max,
  const geometry_msgs::msg::Twist & curr_speed,
  double carrot_gx,
  double carrot_gy) const
{
  Candidate cand;
  cand.kappa = kappa_test;

  const double max_kappa = std::tan(deg2rad(max_steer_deg_)) / std::max(1e-3, wheelbase_);
  cand.kappa = std::clamp(cand.kappa, -max_kappa, max_kappa);

  // 【优化点3】速度上限 = 接近终点上限 + 曲率限速（阿克曼关键）
  const double v_curve_lim = computeCurvatureSpeedLimit(std::fabs(cand.kappa));
  double v_des = std::min(std::clamp(v_des_max, 0.0, max_speed_), v_curve_lim);

  if (v_des > 1e-3) {
    v_des = std::max(v_des, min_speed_);
  } else {
    v_des = 0.0;
  }

  cand.v = v_sign * v_des;

  // accel/decel limit（让速度变化更“稳”）
  const double dt = control_duration_;
  const double v_min = curr_speed.linear.x - max_decel_ * dt;
  const double v_max = curr_speed.linear.x + max_accel_ * dt;
  cand.v = std::clamp(cand.v, v_min, v_max);

  // forward simulate in global frame (map)
  double x = robot_pose_global.pose.position.x;
  double y = robot_pose_global.pose.position.y;
  double th = tf2::getYaw(robot_pose_global.pose.orientation);

  const int steps = std::max(1, static_cast<int>(std::ceil(horizon_time_ / std::max(1e-3, sim_dt_))));
  double sum_fp_cost = 0.0;
  double max_fp_cost = 0.0;

  for (int i = 0; i < steps; ++i) {
    const double omega = cand.v * cand.kappa;
    x  += sim_dt_ * (cand.v * std::cos(th));
    y  += sim_dt_ * (cand.v * std::sin(th));
    th += sim_dt_ * omega;

    if (inCollision(x, y, th)) {
      cand.valid = false;
      cand.score = 1e9;
      return cand;
    }

    double fp_cost = collision_checker_->footprintCostAtPose(
      x, y, th, costmap_ros_->getRobotFootprint());

    if (fp_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
      fp_cost = costmap_ros_->getLayeredCostmap()->isTrackingUnknown() ?
        0.0 : static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
    }

    // 【优化点5】高代价直接判无效：减少“贴边蹭墙”
    if (fp_cost >= forbidden_cost_threshold_) {
      cand.valid = false;
      cand.score = 1e9;
      return cand;
    }

    sum_fp_cost += fp_cost;
    max_fp_cost = std::max(max_fp_cost, fp_cost);
  }

  const double avg_fp_cost = sum_fp_cost / steps;

  // 代价压缩：避免膨胀层数值把评分拉爆
  const auto squash = [](double cost) -> double {
    const double xn = std::clamp(cost, 0.0, 254.0) / 254.0;
    return std::sqrt(xn);
  };

  double score = 0.0;

  // max 防“贴边”，avg 防“抖动”
  const double cost_mix = 0.7 * squash(max_fp_cost) + 0.3 * squash(avg_fp_cost);
  score += w_cost_ * cost_mix;

  // 偏离名义曲率惩罚（保持跟踪性）
  score += w_kappa_ * std::fabs(cand.kappa - kappa_nom);
  
  double delta_cmd = std:atan(wheelbase_ * best.kappa);

  // 转向变化惩罚（减少蛇形）
  if (has_prev_kappa_) {
    const double delta_now  = std::atan(wheelbase_ * cand.kappa);
    const double delta_prev = std::atan(wheelbase_ * prev_kappa_);
    score += w_steer_change_ * std::fabs(delta_now - delta_prev);
  }

  // 【优化点4】进度项：预测末端离 carrot 越近越好（避免“最安全=停住”）
  if (w_progress_ > 1e-9) {
    const double dist_to_carrot = std::hypot(x - carrot_gx, y - carrot_gy);
    score += w_progress_ * dist_to_carrot;
  }

  // 倒车惩罚
  if (cand.v < -1e-3) {
    score += w_reverse_;
  }

  cand.valid = true;
  cand.score = score;
  return cand;
}

geometry_msgs::msg::TwistStamped JetsonAckermannController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = clock_->now();
  const double dt = control_duration_;

  // 【优化点7】倒车hold也遵守加减速约束：避免“猛刹猛倒”
  if (reverse_active_ && now < reverse_until_) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;

    const double target_v = -std::fabs(reverse_speed_);
    const double v_min = speed.linear.x - max_decel_ * dt;
    const double v_max = speed.linear.x + max_accel_ * dt;
    cmd.twist.linear.x  = std::clamp(target_v, v_min, v_max);

    cmd.twist.angular.z = 0.0;
    return cmd;
  } else {
    reverse_active_ = false;
  }

  // lock costmap
  nav2_costmap_2d::Costmap2D * cm = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> cm_lock(*(cm->getMutex()));

  auto transformed = transformGlobalPlan(pose);

  // 【优化点1】dist_to_goal 用“真实 global goal”（global_plan_.back），而不是局部截断段末端
  // 你原来 transformPose/plan frame 处理方式同样保留（参考你的 transformPose 实现 :contentReference[oaicite:3]{index=3}）
  geometry_msgs::msg::PoseStamped robot_in_plan;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_in_plan)) {
    throw nav2_core::PlannerException("JetsonAckermannController: can't transform robot pose to plan frame");
  }
  const auto & goal_in_plan = global_plan_.poses.back();
  const double dist_to_goal =
    nav2_util::geometry_utils::euclidean_distance(robot_in_plan, goal_in_plan);

  const double v_des_max = computeApproachSpeedLimit(dist_to_goal);
  if (v_des_max <= 1e-6) {
    has_prev_kappa_ = false;
    no_valid_cmd_count_ = 0;
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.twist.linear.x  = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // lookahead & nominal curvature
  const double lookahead = getLookAheadDistance(speed);
  const auto carrot_base = getLookAheadPoint(lookahead, transformed);

  const double cx = carrot_base.pose.position.x;
  const double cy = carrot_base.pose.position.y;
  const double dist2 = cx * cx + cy * cy;

  double kappa_nom = 0.0;
  if (dist2 > 1e-4) {
    kappa_nom = 2.0 * cy / dist2;  // pure pursuit curvature in base frame
  }

  // decide direction
  double v_sign = 1.0;
  if (allow_reversing_ && cx < 0.0) {
    v_sign = -1.0;
  }

  if (has_prev_kappa_ && (v_sign != prev_v_sign_)) {
    has_prev_kappa_ = false;
  }
  prev_v_sign_ = v_sign;

  // 【优化点4】carrot（base_link）转全局(map)，用于进度项
  double carrot_gx = pose.pose.position.x;
  double carrot_gy = pose.pose.position.y;
  {
    geometry_msgs::msg::PoseStamped carrot_in_base;
    carrot_in_base.header.frame_id = costmap_ros_->getBaseFrameID(); // base_link
    carrot_in_base.header.stamp = pose.header.stamp;
    carrot_in_base.pose = carrot_base.pose;

    geometry_msgs::msg::PoseStamped carrot_in_global;
    if (transformPose(costmap_ros_->getGlobalFrameID(), carrot_in_base, carrot_in_global)) {
      carrot_gx = carrot_in_global.pose.position.x;
      carrot_gy = carrot_in_global.pose.position.y;
    } else {
      // transform失败就退化为平面近似（不至于炸）
      const double th0 = tf2::getYaw(pose.pose.orientation);
      carrot_gx = pose.pose.position.x + cx * std::cos(th0) - cy * std::sin(th0);
      carrot_gy = pose.pose.position.y + cx * std::sin(th0) + cy * std::cos(th0);
    }
  }

  const double max_kappa = std::tan(deg2rad(max_steer_deg_)) / std::max(1e-3, wheelbase_);
  const int half = num_kappa_samples_ / 2;

  // 【优化点2】采样跨度：ratio * max_kappa（减少跳变导致的蛇形）
  const double sample_span = kappa_sample_span_ratio_ * max_kappa;
  const double dk = (2.0 * sample_span) / std::max(1, num_kappa_samples_ - 1);

  Candidate best;
  best.score = 1e9;
  best.valid = false;

  for (int i = -half; i <= half; ++i) {
    double k_test = kappa_nom + i * dk;
    k_test = std::clamp(k_test, -max_kappa, max_kappa);

    auto cand = evaluateCandidate(pose, transformed, kappa_nom, k_test, v_sign, v_des_max, speed, carrot_gx, carrot_gy);
    if (cand.valid && cand.score < best.score) {
      best = cand;
    }
  }

  // 【优化点6】无解处理：先累计无解次数；连续无解N次再倒车，避免误触发
  if (!best.valid) {
    no_valid_cmd_count_++;

    if (allow_reversing_ && no_valid_cmd_count_ >= no_valid_cmd_before_reverse_) {
      auto rec = evaluateCandidate(pose, transformed, kappa_nom, 0.0, -1.0, std::fabs(reverse_speed_), speed, carrot_gx, carrot_gy);
      if (rec.valid) {
        best = rec;
        reverse_active_ = true;
        reverse_until_ = clock_->now() + rclcpp::Duration::from_seconds(reverse_hold_time_);
        no_valid_cmd_count_ = 0;

        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
          "No safe cmd for %d cycles, using reverse recovery hold (with accel limits).",
          no_valid_cmd_before_reverse_);
      }
    }

    if (!best.valid) {
      has_prev_kappa_ = false;
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header = pose.header;
      cmd.twist.linear.x  = 0.0;
      cmd.twist.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "No valid command, output zero cmd (blocked/wait). no_valid_count=%d dist_to_goal=%.2f",
        no_valid_cmd_count_, dist_to_goal);
      return cmd;
    }
  } else {
    no_valid_cmd_count_ = 0;
  }

  prev_kappa_ = best.kappa;
  has_prev_kappa_ = true;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x  = best.v;
  cmd.twist.angular.z = best.v * best.kappa;

  RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 500,
    "cmd v=%.3f kappa=%.3f k_nom=%.3f score=%.3f dist_goal=%.2f",
    cmd.twist.linear.x, best.kappa, kappa_nom, best.score, dist_to_goal);

  return cmd;
}

}  // namespace jetson_ackermann_controller

PLUGINLIB_EXPORT_CLASS(
  jetson_ackermann_controller::JetsonAckermannController,
  nav2_core::Controller)

