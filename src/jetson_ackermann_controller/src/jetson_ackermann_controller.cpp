#include "jetson_ackermann_controller/jetson_ackermann_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace jetson_ackermann_controller
{

static double deg2rad(double d) { return d * M_PI / 180.0; }

static double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
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

// ==============================
// 【横向误差优化】
// 带符号路径跟踪误差。
// 坐标约定：path_in_base 已经在 base_link 坐标系下。
// x_rel, y_rel, th_rel 是预测出来的车辆 base_link 相对当前 base_link 的位姿。
// signed_lat > 0：预测点在路径左侧。
// signed_lat < 0：预测点在路径右侧。
// 对于控制来说：
//   如果 signed_lat < 0，说明路径在车左边，需要给正曲率往左修。
//   如果 signed_lat > 0，说明路径在车右边，需要给负曲率往右修。
// ==============================
struct TrackingError
{
  double signed_lat{0.0};
  double abs_lat{0.0};
  double heading_err{0.0};
  double abs_yaw_err{0.0};
  double path_heading{0.0};
};

static TrackingError computeTrackingErrorSigned(
  const nav_msgs::msg::Path & path_in_base,
  double x_rel,
  double y_rel,
  double th_rel)
{
  TrackingError out;

  if (path_in_base.poses.empty()) {
    return out;
  }

  if (path_in_base.poses.size() == 1) {
    const auto & p = path_in_base.poses.front().pose.position;
    const double path_yaw = tf2::getYaw(path_in_base.poses.front().pose.orientation);

    const double dx = x_rel - p.x;
    const double dy = y_rel - p.y;

    // 把误差投影到路径坐标系，得到有符号横向误差。
    out.signed_lat =
      -std::sin(path_yaw) * dx + std::cos(path_yaw) * dy;
    out.abs_lat = std::hypot(dx, dy);
    out.path_heading = path_yaw;
    out.heading_err = normalizeAngle(path_yaw - th_rel);
    out.abs_yaw_err = std::fabs(out.heading_err);
    return out;
  }

  double best_dist2 = std::numeric_limits<double>::max();
  double best_signed = 0.0;
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

    const double ex = x_rel - proj_x;
    const double ey = y_rel - proj_y;

    const double d2 = ex * ex + ey * ey;

    if (d2 < best_dist2) {
      best_dist2 = d2;

      const double seg_len = std::sqrt(seg_len2);

      // 二维叉乘确定左右侧。
      // signed_lat > 0：点在路径左侧。
      // signed_lat < 0：点在路径右侧。
      best_signed = (vx * ey - vy * ex) / seg_len;
      best_heading = std::atan2(vy, vx);
    }
  }

  if (best_dist2 == std::numeric_limits<double>::max()) {
    return out;
  }

  out.signed_lat = best_signed;
  out.abs_lat = std::sqrt(std::max(0.0, best_dist2));
  out.path_heading = best_heading;
  out.heading_err = normalizeAngle(best_heading - th_rel);
  out.abs_yaw_err = std::fabs(out.heading_err);

  return out;
}

// ==============================
// 【兼容保留】
// 原来的函数保留，但内部改为使用带符号误差函数。
// 这样其他地方如果还调用 computeTrackingErrorsOnPlan，也不会断。
// ==============================
static void computeTrackingErrorsOnPlan(
  const nav_msgs::msg::Path & path_in_base,
  double x_rel,
  double y_rel,
  double th_rel,
  double & lat_err,
  double & yaw_err)
{
  const auto err = computeTrackingErrorSigned(path_in_base, x_rel, y_rel, th_rel);
  lat_err = err.abs_lat;
  yaw_err = err.abs_yaw_err;
}

// ==============================
// BEV 回调：接收 /bev/cost_grid，并转换成内部 Costmap2D
// 要求 /bev/cost_grid 发布在 base_link 坐标系下
// ==============================
void JetsonAckermannController::bevCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(bev_mutex_);

  if (!msg || msg->info.width == 0 || msg->info.height == 0) {
    bev_ready_ = false;
    return;
  }

  const unsigned int size_x = msg->info.width;
  const unsigned int size_y = msg->info.height;

  if (msg->data.size() != static_cast<size_t>(size_x) * size_y) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "BEV grid data size mismatch. data=%zu expected=%u",
      msg->data.size(), size_x * size_y);
    bev_ready_ = false;
    return;
  }

  if (msg->header.frame_id != costmap_ros_->getBaseFrameID()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "BEV grid frame is '%s', but controller base frame is '%s'. "
      "Please publish /bev/cost_grid in base_link frame.",
      msg->header.frame_id.c_str(),
      costmap_ros_->getBaseFrameID().c_str());
    bev_ready_ = false;
    return;
  }

  const double origin_yaw = tf2::getYaw(msg->info.origin.orientation);
  if (std::fabs(normalizeAngle(origin_yaw)) > deg2rad(1.0)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "BEV grid origin yaw is %.2f deg, but controller assumes axis-aligned grid.",
      origin_yaw * 180.0 / M_PI);
  }

  const double resolution = msg->info.resolution;
  const double origin_x = msg->info.origin.position.x;
  const double origin_y = msg->info.origin.position.y;

  if (!bev_costmap_) {
    bev_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      size_x, size_y, resolution, origin_x, origin_y,
      nav2_costmap_2d::FREE_SPACE);
  } else {
    bev_costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      const size_t idx = static_cast<size_t>(my) * size_x + mx;
      const int occ = static_cast<int>(msg->data[idx]);

      unsigned char cost = nav2_costmap_2d::FREE_SPACE;

      if (occ < 0) {
        cost = static_cast<unsigned char>(std::clamp(bev_unknown_cost_, 0.0, 254.0));
      } else {
        cost = static_cast<unsigned char>(
          std::clamp(
            std::round((static_cast<double>(occ) / 100.0) * 254.0),
            0.0, 254.0));
      }

      bev_costmap_->setCost(mx, my, cost);
    }
  }

  if (!bev_collision_checker_) {
    bev_collision_checker_ =
      std::make_unique<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
      bev_costmap_.get());
  } else {
    bev_collision_checker_->setCostmap(bev_costmap_.get());
  }

  bev_stamp_ = rclcpp::Time(msg->header.stamp);
  bev_ready_ = true;
}

bool JetsonAckermannController::bevAvailable() const
{
  std::lock_guard<std::mutex> lock(bev_mutex_);

  if (!bev_ready_ || !bev_costmap_ || !bev_collision_checker_) {
    return false;
  }

  const auto now = clock_->now();
  if ((now - bev_stamp_).seconds() > bev_timeout_) {
    return false;
  }

  return true;
}

double JetsonAckermannController::legacyFootprintCostAtPose(
  double x, double y, double theta) const
{
  double fp_cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());

  if (fp_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    fp_cost = costmap_ros_->getLayeredCostmap()->isTrackingUnknown() ?
      0.0 : static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  return fp_cost;
}

double JetsonAckermannController::bevFootprintCostAtPose(
  double x, double y, double theta) const
{
  std::lock_guard<std::mutex> lock(bev_mutex_);

  if (!bev_ready_ || !bev_costmap_ || !bev_collision_checker_) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  double fp_cost = bev_collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());

  if (fp_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    fp_cost = std::clamp(bev_unknown_cost_, 0.0, 254.0);
  }

  return fp_cost;
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

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reverse_hold_time", rclcpp::ParameterValue(0.8));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.8));

  // ==============================
  // 【横向误差优化】
  // 这里默认从 45.0 改成 31.86，和底盘真实舵角限制一致。
  // ==============================
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steer_deg", rclcpp::ParameterValue(40.00));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steer_rate_deg_per_s", rclcpp::ParameterValue(80.0));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_speed", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_accel", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_decel", rclcpp::ParameterValue(1.2));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reverse_speed", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".direction_switch_x", rclcpp::ParameterValue(0.25));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead", rclcpp::ParameterValue(1.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_path_resample", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_resample_ds", rclcpp::ParameterValue(0.08));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".num_kappa_samples", rclcpp::ParameterValue(11));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".horizon_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sim_dt", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_cost", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_kappa", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_steer_change", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_reverse", rclcpp::ParameterValue(2.0));

  // ==============================
  // 【横向误差优化】
  // 默认权重提高，因为你现在的问题就是 base_link 弯道偏离路径。
  // 注意：下面的误差在 evaluateCandidate 里按平方惩罚。
  // ==============================
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_lat_error", rclcpp::ParameterValue(12.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_yaw_error", rclcpp::ParameterValue(2.0));

  // ==============================
  // 【横向误差优化】
  // 新增横向误差反馈参数。
  // path_lat_kp 越大，base_link 越贴路径，但太大会蛇形。
  // path_yaw_kp 越大，车头越努力对齐路径方向。
  // path_lat_deadband 用来滤掉 1cm 以内的小抖动。
  // ==============================
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_lat_kp", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_yaw_kp", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_lat_deadband", rclcpp::ParameterValue(0.01));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.06));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".stop_distance", rclcpp::ParameterValue(0.05));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".kappa_sample_span_ratio", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lat_accel", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_progress", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".forbidden_cost_threshold", rclcpp::ParameterValue(220.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".no_valid_cmd_before_reverse", rclcpp::ParameterValue(5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_adaptive_sampling_span", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sampling_span_min_ratio", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sampling_span_max_ratio", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sampling_span_obstacle_gain", rclcpp::ParameterValue(0.8));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_obstacle_speed_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_speed_cost_gain", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_speed_min_scale", rclcpp::ParameterValue(0.2));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_tf_latest_fallback", rclcpp::ParameterValue(true));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bev_topic",
    rclcpp::ParameterValue(std::string("/bev/cost_grid")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bev_timeout",
    rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bev_unknown_cost",
    rclcpp::ParameterValue(80.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_bev_as_primary",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".bev_fallback_to_costmap",
    rclcpp::ParameterValue(true));

  // ==============================
  // 终端位姿微调默认值
  // ==============================
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_terminal_align", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_align_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_xy_tolerance", rclcpp::ParameterValue(0.066));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_yaw_tolerance_deg", rclcpp::ParameterValue(17.19));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_max_speed", rclcpp::ParameterValue(0.06));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_min_speed", rclcpp::ParameterValue(0.015));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_horizon_time", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_num_kappa_samples", rclcpp::ParameterValue(21));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_w_lon_error", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_w_lat_error", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_w_yaw_error", rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_w_steer_change", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_w_reverse", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_w_direction_switch", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_required_stable_count", rclcpp::ParameterValue(3));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_timeout", rclcpp::ParameterValue(8.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_no_improve_cycles", rclcpp::ParameterValue(160));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_min_score_improve", rclcpp::ParameterValue(0.0001));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_freeze_dist", rclcpp::ParameterValue(0.16));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_leave_goal_margin", rclcpp::ParameterValue(0.04));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_max_kappa_ratio", rclcpp::ParameterValue(0.55));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_allow_reverse", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_reverse_min_x", rclcpp::ParameterValue(0.05));

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
  if (num_kappa_samples_ % 2 == 0) {
    num_kappa_samples_ += 1;
  }

  node->get_parameter(plugin_name_ + ".horizon_time", horizon_time_);
  node->get_parameter(plugin_name_ + ".sim_dt", sim_dt_);
  node->get_parameter(plugin_name_ + ".w_cost", w_cost_);
  node->get_parameter(plugin_name_ + ".w_kappa", w_kappa_);
  node->get_parameter(plugin_name_ + ".w_steer_change", w_steer_change_);
  node->get_parameter(plugin_name_ + ".w_reverse", w_reverse_);
  node->get_parameter(plugin_name_ + ".w_lat_error", w_lat_error_);
  node->get_parameter(plugin_name_ + ".w_yaw_error", w_yaw_error_);

  // ==============================
  // 【横向误差优化】
  // 读取新增参数。
  // ==============================
  node->get_parameter(plugin_name_ + ".path_lat_kp", path_lat_kp_);
  node->get_parameter(plugin_name_ + ".path_yaw_kp", path_yaw_kp_);
  node->get_parameter(plugin_name_ + ".path_lat_deadband", path_lat_deadband_);

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

  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".allow_tf_latest_fallback", allow_tf_latest_fallback_);

  node->get_parameter(plugin_name_ + ".bev_topic", bev_topic_);
  node->get_parameter(plugin_name_ + ".bev_timeout", bev_timeout_);
  node->get_parameter(plugin_name_ + ".bev_unknown_cost", bev_unknown_cost_);
  node->get_parameter(plugin_name_ + ".use_bev_as_primary", use_bev_as_primary_);
  node->get_parameter(plugin_name_ + ".bev_fallback_to_costmap", bev_fallback_to_costmap_);

  double terminal_yaw_tolerance_deg = 10.0;
  node->get_parameter(plugin_name_ + ".enable_terminal_align", enable_terminal_align_);
  node->get_parameter(plugin_name_ + ".terminal_align_dist", terminal_align_dist_);
  node->get_parameter(plugin_name_ + ".terminal_xy_tolerance", terminal_xy_tolerance_);
  node->get_parameter(plugin_name_ + ".terminal_yaw_tolerance_deg", terminal_yaw_tolerance_deg);
  node->get_parameter(plugin_name_ + ".terminal_max_speed", terminal_max_speed_);
  node->get_parameter(plugin_name_ + ".terminal_min_speed", terminal_min_speed_);
  node->get_parameter(plugin_name_ + ".terminal_horizon_time", terminal_horizon_time_);
  node->get_parameter(plugin_name_ + ".terminal_num_kappa_samples", terminal_num_kappa_samples_);

  node->get_parameter(plugin_name_ + ".terminal_w_lon_error", terminal_w_lon_error_);
  node->get_parameter(plugin_name_ + ".terminal_w_lat_error", terminal_w_lat_error_);
  node->get_parameter(plugin_name_ + ".terminal_w_yaw_error", terminal_w_yaw_error_);
  node->get_parameter(plugin_name_ + ".terminal_w_steer_change", terminal_w_steer_change_);
  node->get_parameter(plugin_name_ + ".terminal_w_reverse", terminal_w_reverse_);
  node->get_parameter(plugin_name_ + ".terminal_w_direction_switch", terminal_w_direction_switch_);
  node->get_parameter(plugin_name_ + ".terminal_required_stable_count", terminal_required_stable_count_);

  node->get_parameter(plugin_name_ + ".terminal_timeout", terminal_timeout_);
  node->get_parameter(plugin_name_ + ".terminal_no_improve_cycles", terminal_no_improve_cycles_);
  node->get_parameter(plugin_name_ + ".terminal_min_score_improve", terminal_min_score_improve_);
  node->get_parameter(plugin_name_ + ".terminal_freeze_dist", terminal_freeze_dist_);
  node->get_parameter(plugin_name_ + ".terminal_leave_goal_margin", terminal_leave_goal_margin_);
  node->get_parameter(plugin_name_ + ".terminal_max_kappa_ratio", terminal_max_kappa_ratio_);
  node->get_parameter(plugin_name_ + ".terminal_allow_reverse", terminal_allow_reverse_);
  node->get_parameter(plugin_name_ + ".terminal_reverse_min_x", terminal_reverse_min_x_);

  terminal_yaw_tolerance_ = deg2rad(terminal_yaw_tolerance_deg);

  // ==============================
  // 参数约束
  // ==============================
  kappa_sample_span_ratio_ = std::clamp(kappa_sample_span_ratio_, 0.05, 1.0);
  sampling_span_min_ratio_ = std::clamp(sampling_span_min_ratio_, 0.05, 1.0);
  sampling_span_max_ratio_ =
    std::clamp(sampling_span_max_ratio_, sampling_span_min_ratio_, 1.0);

  max_lat_accel_ = std::max(0.0, max_lat_accel_);
  forbidden_cost_threshold_ = std::clamp(forbidden_cost_threshold_, 0.0, 253.0);
  no_valid_cmd_before_reverse_ = std::max(1, no_valid_cmd_before_reverse_);
  obstacle_speed_min_scale_ = std::clamp(obstacle_speed_min_scale_, 0.05, 1.0);
  max_steer_rate_deg_per_s_ = std::max(1.0, max_steer_rate_deg_per_s_);
  bev_unknown_cost_ = std::clamp(bev_unknown_cost_, 0.0, 254.0);
  transform_tolerance_ = std::max(0.0, transform_tolerance_);

  // ==============================
  // 【横向误差优化】
  // 限制新增反馈参数，防止参数填太大导致车辆蛇形。
  // ==============================
  path_lat_kp_ = std::clamp(path_lat_kp_, 0.0, 10.0);
  path_yaw_kp_ = std::clamp(path_yaw_kp_, 0.0, 5.0);
  path_lat_deadband_ = std::clamp(path_lat_deadband_, 0.0, 0.10);

  terminal_align_dist_ = std::max(0.05, terminal_align_dist_);
  terminal_xy_tolerance_ = std::clamp(terminal_xy_tolerance_, 0.005, 0.30);
  terminal_yaw_tolerance_ = std::clamp(terminal_yaw_tolerance_, deg2rad(0.5), deg2rad(20.0));
  terminal_max_speed_ = std::clamp(terminal_max_speed_, 0.01, max_speed_);
  terminal_min_speed_ = std::clamp(terminal_min_speed_, 0.0, terminal_max_speed_);
  terminal_horizon_time_ = std::clamp(terminal_horizon_time_, 0.2, 5.0);

  terminal_num_kappa_samples_ = std::max(3, terminal_num_kappa_samples_);
  if (terminal_num_kappa_samples_ % 2 == 0) {
    terminal_num_kappa_samples_ += 1;
  }

  terminal_required_stable_count_ = std::max(1, terminal_required_stable_count_);

  terminal_timeout_ = std::clamp(terminal_timeout_, 0.2, 10.0);
  terminal_no_improve_cycles_ = std::max(1, terminal_no_improve_cycles_);
  terminal_min_score_improve_ = std::clamp(terminal_min_score_improve_, 1e-5, 10.0);

  terminal_freeze_dist_ = std::clamp(terminal_freeze_dist_, 0.03, terminal_align_dist_);
  terminal_leave_goal_margin_ = std::clamp(terminal_leave_goal_margin_, 0.0, 0.20);
  terminal_max_kappa_ratio_ = std::clamp(terminal_max_kappa_ratio_, 0.05, 1.0);
  terminal_reverse_min_x_ = std::clamp(terminal_reverse_min_x_, 0.02, 1.0);

  collision_checker_ =
    std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);

  bev_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    bev_topic_,
    rclcpp::QoS(1),
    std::bind(&JetsonAckermannController::bevCallback, this, std::placeholders::_1));

  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;

  terminal_stable_count_ = 0;
  terminal_active_ = false;
  terminal_abort_ = false;
  terminal_last_best_score_ = 1e9;
  terminal_no_improve_count_ = 0;
  terminal_enter_time_ = clock_->now();

  RCLCPP_INFO(
    logger_,
    "JetsonAckermannController configured. "
    "wheelbase=%.3f max_steer=%.1fdeg rate=%.1fdeg/s desired=%.2f max_v=%.2f "
    "terminal_align=%s terminal_dist=%.2f xy_tol=%.3f yaw_tol=%.2fdeg "
    "terminal_v=[%.3f, %.3f] terminal_reverse=%s timeout=%.2f freeze=%.2f kappa_ratio=%.2f "
    "path_lat_kp=%.2f path_yaw_kp=%.2f lat_deadband=%.3f",
    wheelbase_,
    max_steer_deg_,
    max_steer_rate_deg_per_s_,
    desired_speed_,
    max_speed_,
    enable_terminal_align_ ? "true" : "false",
    terminal_align_dist_,
    terminal_xy_tolerance_,
    terminal_yaw_tolerance_ * 180.0 / M_PI,
    terminal_min_speed_,
    terminal_max_speed_,
    terminal_allow_reverse_ ? "true" : "false",
    terminal_timeout_,
    terminal_freeze_dist_,
    terminal_max_kappa_ratio_,
    path_lat_kp_,
    path_yaw_kp_,
    path_lat_deadband_);
}

void JetsonAckermannController::cleanup()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  no_valid_cmd_count_ = 0;

  terminal_stable_count_ = 0;
  terminal_active_ = false;
  terminal_abort_ = false;
  terminal_last_best_score_ = 1e9;
  terminal_no_improve_count_ = 0;
  terminal_enter_time_ = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);

  bev_ready_ = false;
}

void JetsonAckermannController::activate()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  reverse_until_ = clock_->now();
  no_valid_cmd_count_ = 0;

  terminal_stable_count_ = 0;
  terminal_active_ = false;
  terminal_abort_ = false;
  terminal_last_best_score_ = 1e9;
  terminal_no_improve_count_ = 0;
  terminal_enter_time_ = clock_->now();
}

void JetsonAckermannController::deactivate()
{
  has_prev_kappa_ = false;
  prev_kappa_ = 0.0;
  prev_v_sign_ = 1.0;
  reverse_active_ = false;
  no_valid_cmd_count_ = 0;

  terminal_stable_count_ = 0;
  terminal_active_ = false;
  terminal_abort_ = false;
  terminal_last_best_score_ = 1e9;
  terminal_no_improve_count_ = 0;
  terminal_enter_time_ = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
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

  terminal_stable_count_ = 0;
  terminal_active_ = false;
  terminal_abort_ = false;
  terminal_last_best_score_ = 1e9;
  terminal_no_improve_count_ = 0;
  terminal_enter_time_ = clock_->now();
}

void JetsonAckermannController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
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
  const double max_dim =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
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
    tf_->transform(
      in, out, frame,
      tf2::durationFromSec(transform_tolerance_));
    out.header.frame_id = frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "transformPose exact-time failed, will %s latest TF. reason: %s",
      allow_tf_latest_fallback_ ? "fallback to" : "NOT fallback to",
      ex.what());
  }

  if (!allow_tf_latest_fallback_) {
    RCLCPP_ERROR(logger_, "transformPose failed and latest fallback disabled.");
    return false;
  }

  try {
    geometry_msgs::msg::PoseStamped in_latest = in;
    in_latest.header.stamp.sec = 0;
    in_latest.header.stamp.nanosec = 0;

    tf_->transform(
      in_latest, out, frame,
      tf2::durationFromSec(transform_tolerance_));

    out.header.frame_id = frame;
    out.header.stamp = in.header.stamp;
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
    throw nav2_core::PlannerException(
      "JetsonAckermannController: can't transform robot pose to plan frame");
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

double JetsonAckermannController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed) const
{
  if (use_velocity_scaled_lookahead_) {
    const double la = std::fabs(speed.linear.x) * lookahead_time_;
    return std::clamp(la, min_lookahead_, max_lookahead_);
  }

  return std::clamp(lookahead_dist_, min_lookahead_, max_lookahead_);
}

geometry_msgs::msg::PoseStamped JetsonAckermannController::getLookAheadPoint(
  const double lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan) const
{
  auto it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(),
    [&](const auto & ps) {
      return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  if (it == transformed_plan.poses.end()) {
    return transformed_plan.poses.back();
  }

  return *it;
}

bool JetsonAckermannController::inCollision(
  const double x,
  const double y,
  const double theta) const
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
    const double scale =
      std::clamp(dist_to_goal / approach_velocity_scaling_dist_, 0.0, 1.0);
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

JetsonAckermannController::Candidate
JetsonAckermannController::evaluateCandidate(
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

  const double max_kappa =
    std::tan(deg2rad(max_steer_deg_)) / std::max(1e-3, wheelbase_);
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

  const int steps = std::max(
    1, static_cast<int>(std::ceil(horizon_time_ / std::max(1e-3, sim_dt_))));
  double sum_fp_cost = 0.0;
  double max_fp_cost = 0.0;

  for (int i = 0; i < steps; ++i) {
    const double omega = cand.v * cand.kappa;

    x += sim_dt_ * (cand.v * std::cos(th));
    y += sim_dt_ * (cand.v * std::sin(th));
    th = normalizeAngle(th + sim_dt_ * omega);

    x_rel += sim_dt_ * (cand.v * std::cos(th_rel));
    y_rel += sim_dt_ * (cand.v * std::sin(th_rel));
    th_rel = normalizeAngle(th_rel + sim_dt_ * omega);

    double fp_cost = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
    bool have_cost = false;

    if (use_bev_as_primary_ && bevAvailable()) {
      fp_cost = bevFootprintCostAtPose(x_rel, y_rel, th_rel);
      have_cost = true;
    } else if (!use_bev_as_primary_ || bev_fallback_to_costmap_) {
      fp_cost = legacyFootprintCostAtPose(x, y, th);
      have_cost = true;
    }

    if (!have_cost) {
      fp_cost = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
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
    const double delta_now = std::atan(wheelbase_ * cand.kappa);
    const double delta_prev = std::atan(wheelbase_ * prev_kappa_);
    score += w_steer_change_ * std::fabs(delta_now - delta_prev);
  }

  // ==============================
  // 【横向误差优化】
  // 原来这里用无符号 lat_err + 一次项：
  //   score += w_lat_error_ * lat_err;
  //   score += w_yaw_error_ * yaw_err;
  //
  // 现在改成：
  //   1. 用新的带符号误差函数计算误差。
  //   2. 评分使用 abs_lat 和 abs_yaw_err。
  //   3. 使用平方惩罚，让偏离路径较大时惩罚明显变大。
  //
  // 这样弯道时，采样器会更偏向选择能把 base_link 拉回路径的曲率。
  // ==============================
  const auto err = computeTrackingErrorSigned(
    transformed_plan_base, x_rel, y_rel, th_rel);

  score += w_lat_error_ * (
    0.7 * err.abs_lat +
    0.3 * err.abs_lat * err.abs_lat
  );

  score += w_yaw_error_ * (
    0.5 * err.abs_yaw_err +
    0.5 * err.abs_yaw_err * err.abs_yaw_err
  );

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

// ==============================
// 终端位姿微调候选评价
// ==============================
JetsonAckermannController::Candidate
JetsonAckermannController::evaluateTerminalCandidate(
  const geometry_msgs::msg::PoseStamped & robot_pose_global,
  double goal_x_base,
  double goal_y_base,
  double goal_yaw_base,
  double kappa_test,
  double v_sign,
  double v_des,
  const geometry_msgs::msg::Twist & curr_speed) const
{
  Candidate cand;

  const double raw_max_kappa =
    std::tan(deg2rad(max_steer_deg_)) / std::max(1e-3, wheelbase_);
  const double max_kappa =
    std::max(1e-4, terminal_max_kappa_ratio_ * raw_max_kappa);

  cand.kappa = std::clamp(kappa_test, -max_kappa, max_kappa);

  double v_abs = std::clamp(v_des, 0.0, terminal_max_speed_);

  if (v_sign < 0.0) {
    v_abs = std::min(v_abs, std::fabs(reverse_speed_));
    v_abs = std::min(v_abs, terminal_max_speed_);
  }

  cand.v = v_sign * v_abs;

  const double dt = control_duration_;
  const double v_min = curr_speed.linear.x - max_decel_ * dt;
  const double v_max = curr_speed.linear.x + max_accel_ * dt;
  cand.v = std::clamp(cand.v, v_min, v_max);

  double x_global = robot_pose_global.pose.position.x;
  double y_global = robot_pose_global.pose.position.y;
  double th_global = tf2::getYaw(robot_pose_global.pose.orientation);

  double x_rel = 0.0;
  double y_rel = 0.0;
  double th_rel = 0.0;

  const int steps = std::max(
    1,
    static_cast<int>(std::ceil(terminal_horizon_time_ / std::max(1e-3, sim_dt_))));

  double sum_fp_cost = 0.0;
  double max_fp_cost = 0.0;

  for (int i = 0; i < steps; ++i) {
    const double omega = cand.v * cand.kappa;

    x_global += sim_dt_ * cand.v * std::cos(th_global);
    y_global += sim_dt_ * cand.v * std::sin(th_global);
    th_global = normalizeAngle(th_global + sim_dt_ * omega);

    x_rel += sim_dt_ * cand.v * std::cos(th_rel);
    y_rel += sim_dt_ * cand.v * std::sin(th_rel);
    th_rel = normalizeAngle(th_rel + sim_dt_ * omega);

    double fp_cost = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
    bool have_cost = false;

    if (use_bev_as_primary_ && bevAvailable()) {
      fp_cost = bevFootprintCostAtPose(x_rel, y_rel, th_rel);
      have_cost = true;
    } else if (!use_bev_as_primary_ || bev_fallback_to_costmap_) {
      fp_cost = legacyFootprintCostAtPose(x_global, y_global, th_global);
      have_cost = true;
    }

    if (!have_cost) {
      fp_cost = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
    }

    if (fp_cost >= forbidden_cost_threshold_) {
      cand.valid = false;
      cand.score = 1e9;
      return cand;
    }

    sum_fp_cost += fp_cost;
    max_fp_cost = std::max(max_fp_cost, fp_cost);
  }

  const double dx = x_rel - goal_x_base;
  const double dy = y_rel - goal_y_base;

  const double pred_dist = std::hypot(dx, dy);
  const double current_dist = std::hypot(goal_x_base, goal_y_base);

  if (current_dist <= terminal_freeze_dist_ &&
      pred_dist > current_dist + terminal_leave_goal_margin_) {
    cand.valid = false;
    cand.score = 1e9;
    return cand;
  }

  const double lon_err =
    std::cos(goal_yaw_base) * dx + std::sin(goal_yaw_base) * dy;
  const double lat_err =
    -std::sin(goal_yaw_base) * dx + std::cos(goal_yaw_base) * dy;
  const double yaw_err = normalizeAngle(th_rel - goal_yaw_base);

  const auto squash = [](double cost) -> double {
      const double xn = std::clamp(cost, 0.0, 254.0) / 254.0;
      return std::sqrt(xn);
    };

  const double avg_fp_cost = sum_fp_cost / steps;
  const double cost_mix = 0.7 * squash(max_fp_cost) + 0.3 * squash(avg_fp_cost);

  double w_lon = terminal_w_lon_error_;
  double w_lat = terminal_w_lat_error_;

  if (current_dist <= terminal_freeze_dist_) {
    w_lon *= 2.0;
    w_lat *= 2.0;
  }

  double score = 0.0;
  score += w_cost_ * cost_mix;
  score += w_lon * std::fabs(lon_err);
  score += w_lat * std::fabs(lat_err);
  score += terminal_w_yaw_error_ * std::fabs(yaw_err);

  if (has_prev_kappa_) {
    const double delta_now = std::atan(wheelbase_ * cand.kappa);
    const double delta_prev = std::atan(wheelbase_ * prev_kappa_);
    score += terminal_w_steer_change_ * std::fabs(delta_now - delta_prev);
  }

  if (cand.v < -1e-3) {
    score += terminal_w_reverse_;
  }

  const double cand_sign = cand.v >= 0.0 ? 1.0 : -1.0;
  if (has_prev_kappa_ && cand_sign != prev_v_sign_) {
    score += terminal_w_direction_switch_;
  }

  cand.valid = true;
  cand.score = score;
  return cand;
}

geometry_msgs::msg::TwistStamped
JetsonAckermannController::computeTerminalAlignCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  double goal_x_base,
  double goal_y_base,
  double goal_yaw_base)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;

  const auto now = clock_->now();

  if (!terminal_active_) {
    terminal_active_ = true;
    terminal_abort_ = false;
    terminal_enter_time_ = now;
    terminal_last_best_score_ = 1e9;
    terminal_no_improve_count_ = 0;
    terminal_stable_count_ = 0;
  }

  const double dist = std::hypot(goal_x_base, goal_y_base);
  const double yaw_err = std::fabs(normalizeAngle(goal_yaw_base));

  const bool xy_ok = dist <= terminal_xy_tolerance_;
  const bool yaw_ok = yaw_err <= terminal_yaw_tolerance_;
  const bool speed_ok =
    std::fabs(speed.linear.x) < 0.03 &&
    std::fabs(speed.angular.z) < 0.05;

  if (xy_ok && yaw_ok && speed_ok) {
    terminal_stable_count_++;
  } else {
    terminal_stable_count_ = 0;
  }

  if (terminal_stable_count_ >= terminal_required_stable_count_) {
    has_prev_kappa_ = false;
    no_valid_cmd_count_ = 0;

    terminal_active_ = false;
    terminal_abort_ = false;
    terminal_last_best_score_ = 1e9;
    terminal_no_improve_count_ = 0;

    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  const double raw_max_kappa =
    std::tan(deg2rad(max_steer_deg_)) / std::max(1e-3, wheelbase_);
  const double max_kappa =
    std::max(1e-4, terminal_max_kappa_ratio_ * raw_max_kappa);

  double v_des = terminal_max_speed_;

  if (max_decel_ > 1e-6) {
    const double brake_dist = std::max(0.0, dist - terminal_xy_tolerance_);
    const double v_brake = std::sqrt(2.0 * max_decel_ * brake_dist);
    v_des = std::min(v_des, v_brake);
  }

  const double yaw_only_speed =
    std::min(terminal_max_speed_, std::max(0.01, 0.5 * terminal_min_speed_));

  if (!xy_ok) {
    v_des = std::max(v_des, terminal_min_speed_);
  } else if (!yaw_ok) {
    v_des = std::max(v_des, yaw_only_speed);
  }

  v_des = std::clamp(v_des, 0.0, terminal_max_speed_);

  Candidate best;
  best.valid = false;
  best.score = 1e9;

  const int n = std::max(3, terminal_num_kappa_samples_);
  const double dk = (n > 1) ? (2.0 * max_kappa / static_cast<double>(n - 1)) : 0.0;

  std::vector<double> signs;
  signs.push_back(1.0);

  if (terminal_allow_reverse_ && allow_reversing_) {
    signs.push_back(-1.0);
  }

  for (double v_sign : signs) {
    for (int i = 0; i < n; ++i) {
      const double kappa = -max_kappa + i * dk;

      auto cand = evaluateTerminalCandidate(
        pose,
        goal_x_base,
        goal_y_base,
        goal_yaw_base,
        kappa,
        v_sign,
        v_des,
        speed);

      if (cand.valid && cand.score < best.score) {
        best = cand;
      }
    }
  }

  if (!best.valid) {
    terminal_abort_ = true;
    terminal_active_ = false;
    has_prev_kappa_ = false;
    no_valid_cmd_count_ = 0;

    RCLCPP_WARN(
      logger_,
      "Terminal align no valid command, output zero cmd. dist=%.3f yaw_err=%.2fdeg",
      dist,
      yaw_err * 180.0 / M_PI);

    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  if (terminal_last_best_score_ - best.score > terminal_min_score_improve_) {
    terminal_last_best_score_ = best.score;
    terminal_no_improve_count_ = 0;
  } else {
    terminal_no_improve_count_++;
  }

  const bool timeout =
    (now - terminal_enter_time_).seconds() > terminal_timeout_;
  const bool no_improve =
    terminal_no_improve_count_ >= terminal_no_improve_cycles_;

  if (timeout || no_improve) {
    terminal_abort_ = true;
    terminal_active_ = false;
    has_prev_kappa_ = false;
    no_valid_cmd_count_ = 0;

    RCLCPP_WARN(
      logger_,
      "Terminal align stopped. timeout=%s no_improve=%s dist=%.3f yaw_err=%.2fdeg best_score=%.3f",
      timeout ? "true" : "false",
      no_improve ? "true" : "false",
      dist,
      yaw_err * 180.0 / M_PI,
      best.score);

    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  if (has_prev_kappa_) {
    double delta_cmd = std::atan(wheelbase_ * best.kappa);
    const double delta_prev = std::atan(wheelbase_ * prev_kappa_);
    const double max_ddelta =
      deg2rad(max_steer_rate_deg_per_s_) * control_duration_;

    delta_cmd = std::clamp(delta_cmd, delta_prev - max_ddelta, delta_prev + max_ddelta);
    delta_cmd = std::clamp(delta_cmd, -deg2rad(max_steer_deg_), deg2rad(max_steer_deg_));
    best.kappa = std::tan(delta_cmd) / std::max(1e-3, wheelbase_);
  }

  prev_kappa_ = best.kappa;
  prev_v_sign_ = best.v >= 0.0 ? 1.0 : -1.0;
  has_prev_kappa_ = true;

  cmd.twist.linear.x = best.v;
  cmd.twist.angular.z = best.v * best.kappa;

  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 500,
    "Terminal align cmd v=%.3f kappa=%.3f dist=%.3f yaw_err=%.2fdeg score=%.3f sign=%.1f",
    cmd.twist.linear.x,
    best.kappa,
    dist,
    yaw_err * 180.0 / M_PI,
    best.score,
    prev_v_sign_);

  return cmd;
}

geometry_msgs::msg::TwistStamped
JetsonAckermannController::computeVelocityCommands(
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
    cmd.twist.linear.x = std::clamp(target_v, v_min, v_max);
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
    throw nav2_core::PlannerException(
      "JetsonAckermannController: can't transform robot pose to plan frame");
  }

  const auto & goal_in_plan = global_plan_.poses.back();
  const double dist_to_goal =
    nav2_util::geometry_utils::euclidean_distance(robot_in_plan, goal_in_plan);

  if (enable_terminal_align_) {
    geometry_msgs::msg::PoseStamped final_goal = global_plan_.poses.back();
    final_goal.header.stamp = pose.header.stamp;

    geometry_msgs::msg::PoseStamped goal_in_base;
    if (!transformPose(costmap_ros_->getBaseFrameID(), final_goal, goal_in_base)) {
      throw nav2_core::PlannerException(
        "JetsonAckermannController: can't transform final goal to base frame");
    }

    const double goal_x_base = goal_in_base.pose.position.x;
    const double goal_y_base = goal_in_base.pose.position.y;
    const double goal_yaw_base =
      normalizeAngle(tf2::getYaw(goal_in_base.pose.orientation));

    const double terminal_dist = std::hypot(goal_x_base, goal_y_base);

    if (terminal_dist <= terminal_align_dist_) {
      if (terminal_abort_) {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header = pose.header;
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        return cmd;
      }

      return computeTerminalAlignCommand(
        pose,
        speed,
        goal_x_base,
        goal_y_base,
        goal_yaw_base);
    } else {
      terminal_stable_count_ = 0;
      terminal_active_ = false;
      terminal_abort_ = false;
      terminal_last_best_score_ = 1e9;
      terminal_no_improve_count_ = 0;
    }
  }

  double v_des_max = computeApproachSpeedLimit(dist_to_goal);
  if (v_des_max <= 1e-6) {
    has_prev_kappa_ = false;
    no_valid_cmd_count_ = 0;

    terminal_stable_count_ = 0;
    terminal_active_ = false;
    terminal_abort_ = false;
    terminal_last_best_score_ = 1e9;
    terminal_no_improve_count_ = 0;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  const double curr_theta = tf2::getYaw(pose.pose.orientation);

  double curr_fp_cost = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  bool have_curr_cost = false;

  if (use_bev_as_primary_ && bevAvailable()) {
    curr_fp_cost = bevFootprintCostAtPose(0.0, 0.0, 0.0);
    have_curr_cost = true;
  } else if (!use_bev_as_primary_ || bev_fallback_to_costmap_) {
    curr_fp_cost = legacyFootprintCostAtPose(
      pose.pose.position.x,
      pose.pose.position.y,
      curr_theta);
    have_curr_cost = true;
  }

  if (!have_curr_cost) {
    curr_fp_cost = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
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
    cmd.twist.linear.x = 0.0;
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

  // ==============================
  // 【横向误差优化】
  // 这里是本次最关键的修改。
  //
  // 原来 kappa_nom 只来自 pure pursuit：
  //   kappa_nom = 2 * cy / dist2
  //
  // 这会导致弯道时只追前视点，base_link 可以偏离路径。
  //
  // 现在加入当前 base_link 到路径的带符号横向误差：
  //   1. delta_pp  ：pure pursuit 前视点转角
  //   2. delta_lat ：横向误差修正，路径在左边就往左修，路径在右边就往右修
  //   3. delta_yaw ：航向误差修正，让车头更贴合路径方向
  //
  // 最终新的 kappa_nom 作为采样中心。
  // 这样采样器不是围绕“前视点曲率”采样，而是围绕“贴路径曲率”采样。
  // ==============================
  {
    const double max_steer_rad = deg2rad(max_steer_deg_);

    const auto curr_err = computeTrackingErrorSigned(
      transformed,
      0.0,
      0.0,
      0.0);

    double signed_lat = curr_err.signed_lat;
    if (std::fabs(signed_lat) < path_lat_deadband_) {
      signed_lat = 0.0;
    }

    const double delta_pp =
      std::atan(wheelbase_ * std::clamp(
        kappa_nom,
        -std::tan(max_steer_rad) / std::max(1e-3, wheelbase_),
        std::tan(max_steer_rad) / std::max(1e-3, wheelbase_)));

    // Stanley 类横向误差反馈。
    // signed_lat < 0：路径在车左边，需要正转角，所以前面加负号。
    // 分母加 0.25 是为了低速时不要过度敏感。
    const double v_for_gain = std::max(0.08, std::fabs(speed.linear.x));
    const double delta_lat =
      -std::atan2(path_lat_kp_ * signed_lat, v_for_gain + 0.25);

    // 航向误差反馈。
    // heading_err = path_heading - robot_heading。
    const double delta_yaw =
      path_yaw_kp_ * curr_err.heading_err;

    double delta_nom = delta_pp + delta_lat + delta_yaw;
    delta_nom = std::clamp(delta_nom, -max_steer_rad, max_steer_rad);

    kappa_nom = std::tan(delta_nom) / std::max(1e-3, wheelbase_);

    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 500,
      "Path tracking err signed_lat=%.3f abs_lat=%.3f heading_err=%.2fdeg "
      "delta_pp=%.2fdeg delta_lat=%.2fdeg delta_yaw=%.2fdeg kappa_nom=%.3f",
      signed_lat,
      curr_err.abs_lat,
      curr_err.heading_err * 180.0 / M_PI,
      delta_pp * 180.0 / M_PI,
      delta_lat * 180.0 / M_PI,
      delta_yaw * 180.0 / M_PI,
      kappa_nom);
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

  const double max_kappa =
    std::tan(deg2rad(max_steer_deg_)) / std::max(1e-3, wheelbase_);

  kappa_nom = std::clamp(kappa_nom, -max_kappa, max_kappa);

  double span_ratio = kappa_sample_span_ratio_;
  if (enable_adaptive_sampling_span_) {
    const double c_norm = std::sqrt(clamp01(curr_fp_cost / 254.0));
    const double occ = clamp01(sampling_span_obstacle_gain_ * c_norm);
    const double adapt = sampling_span_max_ratio_ -
      occ * (sampling_span_max_ratio_ - sampling_span_min_ratio_);
    span_ratio = std::clamp(adapt, 0.05, 1.0);
  }

  const double sample_span = span_ratio * max_kappa;

  double k_low = std::clamp(kappa_nom - sample_span, -max_kappa, max_kappa);
  double k_high = std::clamp(kappa_nom + sample_span, -max_kappa, max_kappa);

  if (has_prev_kappa_) {
    const double prev_delta = std::atan(wheelbase_ * prev_kappa_);
    const double max_ddelta =
      deg2rad(max_steer_rate_deg_per_s_) * control_duration_;

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
        reverse_until_ = clock_->now() +
          rclcpp::Duration::from_seconds(reverse_hold_time_);
        no_valid_cmd_count_ = 0;

        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "No safe cmd for %d cycles, using reverse recovery hold.",
          no_valid_cmd_before_reverse_);
      }
    }

    if (!best.valid) {
      has_prev_kappa_ = false;
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header = pose.header;
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
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
    const double max_ddelta =
      deg2rad(max_steer_rate_deg_per_s_) * control_duration_;

    delta_cmd = std::clamp(delta_cmd, delta_prev - max_ddelta, delta_prev + max_ddelta);
    delta_cmd = std::clamp(delta_cmd, -deg2rad(max_steer_deg_), deg2rad(max_steer_deg_));
    best.kappa = std::tan(delta_cmd) / std::max(1e-3, wheelbase_);
  }

  prev_kappa_ = best.kappa;
  has_prev_kappa_ = true;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = best.v;
  cmd.twist.angular.z = best.v * best.kappa;

  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 500,
    "cmd v=%.3f kappa=%.3f k_nom=%.3f score=%.3f dist_goal=%.2f bev=%s",
    cmd.twist.linear.x, best.kappa, kappa_nom, best.score, dist_to_goal,
    (use_bev_as_primary_ && bevAvailable()) ? "ON" : "OFF");

  return cmd;
}

}  // namespace jetson_ackermann_controller

PLUGINLIB_EXPORT_CLASS(
  jetson_ackermann_controller::JetsonAckermannController,
  nav2_core::Controller)

