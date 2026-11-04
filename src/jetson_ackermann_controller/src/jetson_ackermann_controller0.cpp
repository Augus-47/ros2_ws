#include "jetson_ackermann_controller/jetson_ackermann_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace jetson_ackermann_controller
{

static double deg2rad(double d) { return d * M_PI / 180.0; }

static double normalizeAngle(double a)
{
  while (a > M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

static double clamp01(double x)
{
  return std::clamp(x, 0.0, 1.0);
}

static nav_msgs::msg::Path resamplePathByDistance(
  const nav_msgs::msg::Path & path,
  double ds)
{
  if (path.poses.size() < 2 || ds <= 1e-4) {
    return path;
  }

  nav_msgs::msg::Path out;
  out.header = path.header;
  out.poses.clear();

  std::vector<double> acc_len(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p0 = path.poses[i - 1].pose.position;
    const auto & p1 = path.poses[i].pose.position;
    const double seg = std::hypot(p1.x - p0.x, p1.y - p0.y);
    acc_len[i] = acc_len[i - 1] + seg;
  }

  const double total_len = acc_len.back();
  if (total_len <= 1e-6) {
    return path;
  }

  out.poses.push_back(path.poses.front());

  size_t seg_idx = 1;
  for (double s = ds; s < total_len; s += ds) {
    while (seg_idx < acc_len.size() && acc_len[seg_idx] < s) {
      ++seg_idx;
    }
    if (seg_idx >= path.poses.size()) {
      break;
    }

    const size_t i0 = seg_idx - 1;
    const size_t i1 = seg_idx;

    const auto & p0 = path.poses[i0].pose.position;
    const auto & p1 = path.poses[i1].pose.position;

    const double seg_len = std::max(1e-6, acc_len[i1] - acc_len[i0]);
    const double ratio = std::clamp((s - acc_len[i0]) / seg_len, 0.0, 1.0);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p0.x + ratio * (p1.x - p0.x);
    ps.pose.position.y = p0.y + ratio * (p1.y - p0.y);
    ps.pose.position.z = 0.0;

    const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    ps.pose.orientation = tf2::toMsg(q);

    out.poses.push_back(ps);
  }

  out.poses.push_back(path.poses.back());
  return out;
}

static void computeTrackingErrorsOnPlan(
  const nav_msgs::msg::Path & path_in_base,
  double x_rel,
  double y_rel,
  double th_rel,
  double & lat_err,
  double & yaw_err)
{
  lat_err = 0.0;
  yaw_err = 0.0;

  if (path_in_base.poses.empty()) {
    return;
  }

  if (path_in_base.poses.size() == 1) {
    const auto & p = path_in_base.poses.front().pose.position;
    lat_err = std::hypot(x_rel - p.x, y_rel - p.y);
    const double path_yaw = tf2::getYaw(path_in_base.poses.front().pose.orientation);
    yaw_err = std::fabs(normalizeAngle(th_rel - path_yaw));
    return;
  }

  double best_dist2 = std::numeric_limits<double>::max();
  double best_heading = 0.0;

  for (size_t i = 0; i + 1 < path_in_base.poses.size(); ++i) {
    const auto & p0 = path_in_base.poses[i].pose.position;
    const auto & p1 = path_in_base.poses[i + 1].pose.position;

    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double seg_len2 = vx * vx + vy * vy;

    if (seg_len2 < 1e-9) {
      continue;
    }

    const double wx = x_rel - p0.x;
    const double wy = y_rel - p0.y;

    double t = (wx * vx + wy * vy) / seg_len2;
    t = std::clamp(t, 0.0, 1.0);

    const double proj_x = p0.x + t * vx;
    const double proj_y = p0.y + t * vy;

    const double dx = x_rel - proj_x;
    const double dy = y_rel - proj_y;
    const double d2 = dx * dx + dy * dy;

    if (d2 < best_dist2) {
      best_dist2 = d2;
      best_heading = std::atan2(vy, vx);
    }
  }

  lat_err = std::sqrt(std::max(0.0, best_dist2));
  yaw_err = std::fabs(normalizeAngle(th_rel - best_heading));
}

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

  declare_parameter_if_not_declared(node, plugin_name_ + ".reverse_hold_time", rclcpp::ParameterValue(0.8));

  declare_parameter_if_not_declared(node, plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_steer_deg", rclcpp::ParameterValue(45.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_steer_rate_deg_per_s", rclcpp::ParameterValue(50.0));

  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_speed", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_accel", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_decel", rclcpp::ParameterValue(1.2));

  declare_parameter_if_not_declared(node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reverse_speed", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".direction_switch_x", rclcpp::ParameterValue(0.25));

  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_velocity_scaled_lookahead", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead", rclcpp::ParameterValue(1.5));

  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_path_resample", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".path_resample_ds", rclcpp::ParameterValue(0.08));

  declare_parameter_if_not_declared(node, plugin_name_ + ".num_kappa_samples", rclcpp::ParameterValue(11));
  declare_parameter_if_not_declared(node, plugin_name_ + ".horizon_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sim_dt", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_cost", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_kappa", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_steer_change", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_reverse", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_lat_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_yaw_error", rclcpp::ParameterValue(0.4));

  declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.06));
  declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".stop_distance", rclcpp::ParameterValue(0.05));

  declare_parameter_if_not_declared(node, plugin_name_ + ".kappa_sample_span_ratio", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lat_accel", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_progress", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".forbidden_cost_threshold", rclcpp::ParameterValue(220.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".no_valid_cmd_before_reverse", rclcpp::ParameterValue(5));

  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_adaptive_sampling_span", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sampling_span_min_ratio", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sampling_span_max_ratio", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sampling_span_obstacle_gain", rclcpp::ParameterValue(0.8));

  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_obstacle_speed_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_speed_cost_gain", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_speed_min_scale", rclcpp::ParameterValue(0.2));

  node->get_parameter(plugin_name_ + ".reverse_hold_time", reverse_hold_time_);

  node->get_parameter(plugin_name_ + ".wheelbase", wheelbase_);
  node->get_parameter(plugin_name_ + ".max_steer_deg", max_steer_deg_);
  node->get_parameter(plugin_name_ + ".max_steer_rate_deg_per_s", max_steer_rate_deg_per_s_);

  node->get_parameter(plugin_name_ + ".desired_speed", desired_speed_);
  base_desired_speed_ = desired_speed_;

  node->get_parameter(plugin_name_ + ".min_speed", min_speed_);
  node->get_parameter(plugin_name_ + ".max_speed", max_speed_);
  node->get_parameter(plugin_name_ + ".max_accel", max_accel_);
  node->get_parameter(plugin_name_ + ".max_decel", max_decel_);

  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter(plugin_name_ + ".reverse_speed", reverse_speed_);
  node->get_parameter(plugin_name_ + ".direction_switch_x", direction_switch_x_);

  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead", use_velocity_scaled_lookahead_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".min_lookahead", min_lookahead_);
  node->get_parameter(plugin_name_ + ".max_lookahead", max_lookahead_);

  node->get_parameter(plugin_name_ + ".enable_path_resample", enable_path_resample_);
  node->get_parameter(plugin_name_ + ".path_resample_ds", path_resample_ds_);

  node->get_parameter(plugin_name_ + ".num_kappa_samples", num_kappa_samples_);
  num_kappa_samples_ = std::max(3, num_kappa_samples_);
  if (num_kappa_samples_ % 2 == 0) { num_kappa_samples_ += 1; }

  node->get_parameter(plugin_name_ + ".horizon_time", horizon_time_);
  node->get_parameter(plugin_name_ + ".sim_dt", sim_dt_);
  node->get_parameter(plugin_name_ + ".w_cost", w_cost_);
  node->get_parameter(plugin_name_ + ".w_kappa", w_kappa_);
  node->get_parameter(plugin_name_ + ".w_steer_change", w_steer_change_);
  node->get_parameter(plugin_name_ + ".w_reverse", w_reverse_);
  node->get_parameter(plugin_name_ + ".w_lat_error", w_lat_error_);
  node->get_parameter(plugin_name_ + ".w_yaw_error", w_yaw_error_);

  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".stop_distance", stop_distance_);

  node->get_parameter(plugin_name_ + ".kappa_sample_span_ratio", kappa_sample_span_ratio_);
  node->get_parameter(plugin_name_ + ".max_lat_accel", max_lat_accel_);
  node->get_parameter(plugin_name_ + ".w_progress", w_progress_);
  node->get_parameter(plugin_name_ + ".forbidden_cost_threshold", forbidden_cost_threshold_);
  node->get_parameter(plugin_name_ + ".no_valid_cmd_before_reverse", no_valid_cmd_before_reverse_);

  node->get_parameter(plugin_name_ + ".enable_adaptive_sampling_span", enable_adaptive_sampling_span_);
  node->get_parameter(plugin_name_ + ".sampling_span_min_ratio", sampling_span_min_ratio_);
  node->get_parameter(plugin_name_ + ".sampling_span_max_ratio", sampling_span_max_ratio_);
  node->get_parameter(plugin_name_ + ".sampling_span_obstacle_gain", sampling_span_obstacle_gain_);

  node->get_parameter(plugin_name_ + ".enable_obstacle_speed_scaling", enable_obstacle_speed_scaling_);
  node->get_parameter(plugin_name_ + ".obstacle_speed_cost_gain", obstacle_speed_cost_gain_);
  node->get_parameter(plugin_name_ + ".obstacle_speed_min_scale", obstacle_speed_min_scale_);

  kappa_sample_span_ratio_ = std::clamp(kappa_sample_span_ratio_, 0.05, 1.0);
  sampling_span_min_ratio_ = std::clamp(sampling_span_min_ratio_, 0.05, 1.0);
  sampling_span_max_ratio_ = std::clamp(sampling_span_max_ratio_, sampling_span_min_ratio_, 1.0);
  max_lat_accel_ = std::max(0.0, max_lat_accel_);
  forbidden_cost_threshold_ = std::clamp(forbidden_cost_threshold_, 0.0, 253.0);
  no_valid_cmd_before_reverse_ = std::max(1, no_valid_cmd_before_reverse_);
  obstacle_speed_min_scale_ = std::clamp(obstacle_speed_min_scale_, 0.05, 1.0);
  max_steer_rate_deg_per_s_ = std::max(1.0, max_steer_rate_deg_per_s_);

  collision_checker_ =
    std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
  collision_checker_->setCostmap(costmap_);

  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;

  RCLCPP_INFO(
    logger_,
    "JetsonAckermannController configured. wheelbase=%.3f max_steer=%.1fdeg rate=%.1fdeg/s "
    "desired=%.2f max_v=%.2f span_ratio=%.2f a_lat=%.2f w_prog=%.2f "
    "w_lat=%.2f w_yaw=%.2f forbid=%.1f no_valid_before_rev=%d",
    wheelbase_, max_steer_deg_, max_steer_rate_deg_per_s_,
    desired_speed_, max_speed_,
    kappa_sample_span_ratio_, max_lat_accel_, w_progress_,
    w_lat_error_, w_yaw_error_,
    forbidden_cost_threshold_, no_valid_cmd_before_reverse_);
}

void JetsonAckermannController::cleanup()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  no_valid_cmd_count_ = 0;
}

void JetsonAckermannController::activate()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;
}

void JetsonAckermannController::deactivate()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  no_valid_cmd_count_ = 0;
}

void JetsonAckermannController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;

  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;
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
  transformed.header.frame_id = costmap_ros_->getBaseFrameID();
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

  global_plan_.poses.erase(global_plan_.poses.begin(), begin_it);

  if (transformed.poses.empty()) {
    throw nav2_core::PlannerException("JetsonAckermannController: transformed plan empty");
  }

  if (enable_path_resample_) {
    transformed = resamplePathByDistance(transformed, path_resample_ds_);
  }

  return transformed;
}

double JetsonAckermannController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const
{
  (void)speed;
  return std::clamp(lookahead_dist_, min_lookahead_, max_lookahead_);
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
  const nav_msgs::msg::Path & transformed_plan_base,
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

  const double v_curve_lim = computeCurvatureSpeedLimit(std::fabs(cand.kappa));
  double v_des = std::min(std::clamp(v_des_max, 0.0, max_speed_), v_curve_lim);

  if (v_des > 1e-3) {
    v_des = std::max(v_des, min_speed_);
  } else {
    v_des = 0.0;
  }

  cand.v = v_sign * v_des;

  const double dt = control_duration_;
  const double v_min = curr_speed.linear.x - max_decel_ * dt;
  const double v_max = curr_speed.linear.x + max_accel_ * dt;
  cand.v = std::clamp(cand.v, v_min, v_max);

  double x = robot_pose_global.pose.position.x;
  double y = robot_pose_global.pose.position.y;
  double th = tf2::getYaw(robot_pose_global.pose.orientation);

  double x_rel = 0.0;
  double y_rel = 0.0;
  double th_rel = 0.0;

  const int steps = std::max(1, static_cast<int>(std::ceil(horizon_time_ / std::max(1e-3, sim_dt_))));
  double sum_fp_cost = 0.0;
  double max_fp_cost = 0.0;

  for (int i = 0; i < steps; ++i) {
    const double omega = cand.v * cand.kappa;

    x  += sim_dt_ * (cand.v * std::cos(th));
    y  += sim_dt_ * (cand.v * std::sin(th));
    th += sim_dt_ * omega;

    x_rel  += sim_dt_ * (cand.v * std::cos(th_rel));
    y_rel  += sim_dt_ * (cand.v * std::sin(th_rel));
    th_rel += sim_dt_ * omega;
    th_rel = normalizeAngle(th_rel);

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

    if (fp_cost >= forbidden_cost_threshold_) {
      cand.valid = false;
      cand.score = 1e9;
      return cand;
    }

    sum_fp_cost += fp_cost;
    max_fp_cost = std::max(max_fp_cost, fp_cost);
  }

  const double avg_fp_cost = sum_fp_cost / steps;

  const auto squash = [](double cost) -> double {
    const double xn = std::clamp(cost, 0.0, 254.0) / 254.0;
    return std::sqrt(xn);
  };

  double score = 0.0;

  const double cost_mix = 0.7 * squash(max_fp_cost) + 0.3 * squash(avg_fp_cost);
  score += w_cost_ * cost_mix;

  score += w_kappa_ * std::fabs(cand.kappa - kappa_nom);

  if (has_prev_kappa_) {
    const double delta_now  = std::atan(wheelbase_ * cand.kappa);
    const double delta_prev = std::atan(wheelbase_ * prev_kappa_);
    score += w_steer_change_ * std::fabs(delta_now - delta_prev);
  }

  double lat_err = 0.0;
  double yaw_err = 0.0;
  computeTrackingErrorsOnPlan(transformed_plan_base, x_rel, y_rel, th_rel, lat_err, yaw_err);

  score += w_lat_error_ * lat_err;
  score += w_yaw_error_ * yaw_err;

  if (w_progress_ > 1e-9) {
    const double dist_to_carrot = std::hypot(x - carrot_gx, y - carrot_gy);
    score += w_progress_ * dist_to_carrot;
  }

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

  nav2_costmap_2d::Costmap2D * cm = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> cm_lock(*(cm->getMutex()));

  auto transformed = transformGlobalPlan(pose);

  geometry_msgs::msg::PoseStamped robot_in_plan;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_in_plan)) {
    throw nav2_core::PlannerException("JetsonAckermannController: can't transform robot pose to plan frame");
  }

  const auto & goal_in_plan = global_plan_.poses.back();
  const double dist_to_goal =
    nav2_util::geometry_utils::euclidean_distance(robot_in_plan, goal_in_plan);

  double v_des_max = computeApproachSpeedLimit(dist_to_goal);
  if (v_des_max <= 1e-6) {
    has_prev_kappa_ = false;
    no_valid_cmd_count_ = 0;
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.twist.linear.x  = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  const double curr_theta = tf2::getYaw(pose.pose.orientation);
  double curr_fp_cost = collision_checker_->footprintCostAtPose(
    pose.pose.position.x,
    pose.pose.position.y,
    curr_theta,
    costmap_ros_->getRobotFootprint());

  if (curr_fp_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    curr_fp_cost = costmap_ros_->getLayeredCostmap()->isTrackingUnknown() ?
      0.0 : static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  if (enable_obstacle_speed_scaling_) {
    const double c_norm = std::sqrt(clamp01(curr_fp_cost / 254.0));
    const double scale = std::clamp(
      1.0 - obstacle_speed_cost_gain_ * c_norm,
      obstacle_speed_min_scale_,
      1.0);
    v_des_max *= scale;
  }

  if (v_des_max <= 1e-6) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.twist.linear.x  = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  const double lookahead = getLookAheadDistance(speed);
  const auto carrot_base = getLookAheadPoint(lookahead, transformed);

  const double cx = carrot_base.pose.position.x;
  const double cy = carrot_base.pose.position.y;
  const double dist2 = cx * cx + cy * cy;

  double kappa_nom = 0.0;
  if (dist2 > 1e-4) {
    kappa_nom = 2.0 * cy / dist2;
  }

  double v_sign = 1.0;
  if (allow_reversing_) {
    if (cx < -std::fabs(direction_switch_x_)) {
      v_sign = -1.0;
    } else if (cx > std::fabs(direction_switch_x_)) {
      v_sign = 1.0;
    } else {
      v_sign = has_prev_kappa_ ? prev_v_sign_ : 1.0;
    }
  }

  if (has_prev_kappa_ && (v_sign != prev_v_sign_)) {
    has_prev_kappa_ = false;
  }
  prev_v_sign_ = v_sign;

  double carrot_gx = pose.pose.position.x;
  double carrot_gy = pose.pose.position.y;
  {
    geometry_msgs::msg::PoseStamped carrot_in_base;
    carrot_in_base.header.frame_id = costmap_ros_->getBaseFrameID();
    carrot_in_base.header.stamp = pose.header.stamp;
    carrot_in_base.pose = carrot_base.pose;

    geometry_msgs::msg::PoseStamped carrot_in_global;
    if (transformPose(costmap_ros_->getGlobalFrameID(), carrot_in_base, carrot_in_global)) {
      carrot_gx = carrot_in_global.pose.position.x;
      carrot_gy = carrot_in_global.pose.position.y;
    } else {
      const double th0 = tf2::getYaw(pose.pose.orientation);
      carrot_gx = pose.pose.position.x + cx * std::cos(th0) - cy * std::sin(th0);
      carrot_gy = pose.pose.position.y + cx * std::sin(th0) + cy * std::cos(th0);
    }
  }

  const double max_kappa = std::tan(deg2rad(max_steer_deg_)) / std::max(1e-3, wheelbase_);
  const int half = num_kappa_samples_ / 2;

  double span_ratio = kappa_sample_span_ratio_;
  if (enable_adaptive_sampling_span_) {
    const double c_norm = std::sqrt(clamp01(curr_fp_cost / 254.0));
    const double occ = clamp01(sampling_span_obstacle_gain_ * c_norm);
    const double adapt = sampling_span_max_ratio_ -
      occ * (sampling_span_max_ratio_ - sampling_span_min_ratio_);
    span_ratio = std::clamp(kappa_sample_span_ratio_ * adapt, 0.05, 1.0);
  }

  const double sample_span = span_ratio * max_kappa;

  double k_low = std::clamp(kappa_nom - sample_span, -max_kappa, max_kappa);
  double k_high = std::clamp(kappa_nom + sample_span, -max_kappa, max_kappa);

  if (has_prev_kappa_) {
    const double prev_delta = std::atan(wheelbase_ * prev_kappa_);
    const double max_ddelta = deg2rad(max_steer_rate_deg_per_s_) * control_duration_;

    const double delta_min = std::clamp(
      prev_delta - max_ddelta,
      -deg2rad(max_steer_deg_),
      deg2rad(max_steer_deg_));
    const double delta_max = std::clamp(
      prev_delta + max_ddelta,
      -deg2rad(max_steer_deg_),
      deg2rad(max_steer_deg_));

    const double k_rate_low = std::tan(delta_min) / std::max(1e-3, wheelbase_);
    const double k_rate_high = std::tan(delta_max) / std::max(1e-3, wheelbase_);

    k_low = std::max(k_low, std::min(k_rate_low, k_rate_high));
    k_high = std::min(k_high, std::max(k_rate_low, k_rate_high));
  }

  if (k_low > k_high) {
    const double k_mid = std::clamp(kappa_nom, -max_kappa, max_kappa);
    k_low = k_mid;
    k_high = k_mid;
  }

  const double dk = (num_kappa_samples_ > 1) ?
    ((k_high - k_low) / static_cast<double>(num_kappa_samples_ - 1)) : 0.0;

  Candidate best;
  best.score = 1e9;
  best.valid = false;

  for (int i = 0; i < num_kappa_samples_; ++i) {
    double k_test = k_low + i * dk;
    k_test = std::clamp(k_test, -max_kappa, max_kappa);

    auto cand = evaluateCandidate(
      pose, transformed, kappa_nom, k_test, v_sign, v_des_max,
      speed, carrot_gx, carrot_gy);

    if (cand.valid && cand.score < best.score) {
      best = cand;
    }
  }

  if (!best.valid) {
    no_valid_cmd_count_++;

    if (allow_reversing_ && no_valid_cmd_count_ >= no_valid_cmd_before_reverse_) {
      auto rec = evaluateCandidate(
        pose, transformed, kappa_nom, 0.0, -1.0, std::fabs(reverse_speed_),
        speed, carrot_gx, carrot_gy);

      if (rec.valid) {
        best = rec;
        reverse_active_ = true;
        reverse_until_ = clock_->now() + rclcpp::Duration::from_seconds(reverse_hold_time_);
        no_valid_cmd_count_ = 0;

        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
          "No safe cmd for %d cycles, using reverse recovery hold.",
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
        "No valid command, output zero cmd. no_valid_count=%d dist_to_goal=%.2f",
        no_valid_cmd_count_, dist_to_goal);
      return cmd;
    }
  } else {
    no_valid_cmd_count_ = 0;
  }

  if (has_prev_kappa_) {
    double delta_cmd = std::atan(wheelbase_ * best.kappa);
    const double delta_prev = std::atan(wheelbase_ * prev_kappa_);
    const double max_ddelta = deg2rad(max_steer_rate_deg_per_s_) * control_duration_;

    delta_cmd = std::clamp(delta_cmd, delta_prev - max_ddelta, delta_prev + max_ddelta);
    delta_cmd = std::clamp(delta_cmd, -deg2rad(max_steer_deg_), deg2rad(max_steer_deg_));
    best.kappa = std::tan(delta_cmd) / std::max(1e-3, wheelbase_);
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

