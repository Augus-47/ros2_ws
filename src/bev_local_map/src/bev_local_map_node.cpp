#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class BevLocalMapNode : public rclcpp::Node
{
public:
  // 构造函数：读取参数、创建订阅与发布器、初始化 TF
  BevLocalMapNode()
  : Node("bev_local_map_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ----------------------------
    // 基本输入输出参数
    // ----------------------------
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/points_filtered");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");

    publish_raw_topic_ = this->declare_parameter<std::string>(
      "publish_raw_topic", "/bev/occupancy_raw");
    publish_cost_topic_ = this->declare_parameter<std::string>(
      "publish_cost_topic", "/bev/cost_grid");

    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.05);

    // ----------------------------
    // BEV 范围与分辨率
    // ----------------------------
    x_min_ = this->declare_parameter<double>("x_min", -3.0);
    x_max_ = this->declare_parameter<double>("x_max", 12.0);
    y_min_ = this->declare_parameter<double>("y_min", -6.0);
    y_max_ = this->declare_parameter<double>("y_max", 6.0);
    resolution_ = this->declare_parameter<double>("resolution", 0.05);

    z_min_ = this->declare_parameter<double>("z_min", -0.30);
    z_max_ = this->declare_parameter<double>("z_max", 1.50);

    // ----------------------------
    // 栅格判定参数
    // ----------------------------
    min_points_per_cell_ = this->declare_parameter<int>("min_points_per_cell", 2);

    flat_height_diff_thresh_ = this->declare_parameter<double>("flat_height_diff_thresh", 0.08);
    flat_z_max_thresh_ = this->declare_parameter<double>("flat_z_max_thresh", 0.15);

    obstacle_height_diff_thresh_ =
      this->declare_parameter<double>("obstacle_height_diff_thresh", 0.12);
    obstacle_z_max_thresh_ =
      this->declare_parameter<double>("obstacle_z_max_thresh", 0.20);

    // ----------------------------
    // OccupancyGrid 代价值
    // ----------------------------
    unknown_occ_value_ = this->declare_parameter<int>("unknown_occ_value", 50);
    rough_occ_value_ = this->declare_parameter<int>("rough_occ_value", 70);
    obstacle_occ_value_ = this->declare_parameter<int>("obstacle_occ_value", 100);
    free_occ_value_ = this->declare_parameter<int>("free_occ_value", 0);

    inflate_radius_ = this->declare_parameter<double>("inflate_radius", 0.25);

    // ----------------------------
    // 多帧融合参数
    // ----------------------------
    enable_temporal_fusion_ =
      this->declare_parameter<bool>("enable_temporal_fusion", true);
    fusion_frame_count_ =
      this->declare_parameter<int>("fusion_frame_count", 4);
    fusion_time_window_ =
      this->declare_parameter<double>("fusion_time_window", 0.4);
    near_range_no_fusion_ =
      this->declare_parameter<double>("near_range_no_fusion", 4.0);

    // ----------------------------
    // 地图尺寸
    // ----------------------------
    grid_width_ =
      static_cast<unsigned int>(std::ceil((x_max_ - x_min_) / resolution_));
    grid_height_ =
      static_cast<unsigned int>(std::ceil((y_max_ - y_min_) / resolution_));

    // 发布器
    raw_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(publish_raw_topic_, 1);
    cost_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(publish_cost_topic_, 1);

    // 订阅点云
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BevLocalMapNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "BEV local map node started. cloud=%s target_frame=%s odom_frame=%s grid=%u x %u res=%.3f",
      cloud_topic_.c_str(), target_frame_.c_str(), odom_frame_.c_str(),
      grid_width_, grid_height_, resolution_);

    RCLCPP_INFO(
      this->get_logger(),
      "Temporal fusion: %s, frames=%d, time_window=%.2f, near_no_fusion=%.2f",
      enable_temporal_fusion_ ? "ON" : "OFF",
      fusion_frame_count_, fusion_time_window_, near_range_no_fusion_);
  }

private:
  // ============================================================
  // 数据结构定义
  // ============================================================

  // 每个栅格里统计的信息：点数、最低高度、最高高度
  struct CellStat
  {
    int count = 0;
    float z_min = std::numeric_limits<float>::infinity();
    float z_max = -std::numeric_limits<float>::infinity();
  };

  // 缓存的一帧点云及其对应的 odom 位姿
  struct TimedCloud
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base;
    geometry_msgs::msg::Pose pose_in_odom;
    rclcpp::Time stamp;

    TimedCloud()
    : cloud_in_base(new pcl::PointCloud<pcl::PointXYZ>()), stamp(0, 0, RCL_ROS_TIME)
    {}
  };

  // ============================================================
  // 函数：查询某个时间戳下 base_link 在 odom 坐标系下的位姿
  // 用途：做多帧融合时的位姿补偿
  // ============================================================
  bool lookupBasePoseInOdom(
    const rclcpp::Time & stamp,
    geometry_msgs::msg::Pose & pose_out)
  {
    try {
      auto tf_msg = tf_buffer_.lookupTransform(
        odom_frame_,
        target_frame_,
        stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));

      pose_out.position.x = tf_msg.transform.translation.x;
      pose_out.position.y = tf_msg.transform.translation.y;
      pose_out.position.z = tf_msg.transform.translation.z;
      pose_out.orientation = tf_msg.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "lookupBasePoseInOdom failed: %s", ex.what());
      return false;
    }
  }

  // ============================================================
  // 函数：把 geometry_msgs::msg::Pose 转成 Eigen 变换矩阵
  // 用途：后面做历史点云的位姿变换
  // ============================================================
  Eigen::Affine3f poseMsgToEigen(const geometry_msgs::msg::Pose & pose) const
  {
    Eigen::Translation3f t(
      static_cast<float>(pose.position.x),
      static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z));

    Eigen::Quaternionf q(
      static_cast<float>(pose.orientation.w),
      static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y),
      static_cast<float>(pose.orientation.z));

    return t * q;
  }

  // ============================================================
  // 函数：清理过旧的点云缓存
  // 用途：只保留时间窗口内、数量上限内的历史帧
  // ============================================================
  void pruneCloudBuffer(const rclcpp::Time & current_stamp)
  {
    while (!cloud_buffer_.empty()) {
      const double age = (current_stamp - cloud_buffer_.front().stamp).seconds();

      if (age > fusion_time_window_ ||
        static_cast<int>(cloud_buffer_.size()) > fusion_frame_count_)
      {
        cloud_buffer_.pop_front();
      } else {
        break;
      }
    }

    while (static_cast<int>(cloud_buffer_.size()) > fusion_frame_count_) {
      cloud_buffer_.pop_front();
    }
  }

  // ============================================================
  // 函数：把历史帧点云补偿到“当前时刻的 base_link”
  // 用途：构造融合后的点云，提高远处 BEV 的完整性
  //
  // 思路：
  //   p_now = T_now^{-1} * T_old * p_old
  //
  // 其中：
  //   p_old 是“历史帧自己的 base_link”下的点
  //   p_now 是“当前帧 base_link”下的点
  // ============================================================
  pcl::PointCloud<pcl::PointXYZ>::Ptr buildFusedCloud(
    const geometry_msgs::msg::Pose & current_pose_in_odom)
  {
    auto fused = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());

    if (cloud_buffer_.empty()) {
      return fused;
    }

    const Eigen::Affine3f T_odom_base_now = poseMsgToEigen(current_pose_in_odom);
    const Eigen::Affine3f T_base_now_odom = T_odom_base_now.inverse();

    for (size_t i = 0; i < cloud_buffer_.size(); ++i) {
      const auto & item = cloud_buffer_[i];
      const bool is_current_frame = (i + 1 == cloud_buffer_.size());

      const Eigen::Affine3f T_odom_base_old = poseMsgToEigen(item.pose_in_odom);
      const Eigen::Affine3f T_base_now_base_old = T_base_now_odom * T_odom_base_old;

      for (const auto & pt_old : item.cloud_in_base->points) {
        if (!std::isfinite(pt_old.x) || !std::isfinite(pt_old.y) || !std::isfinite(pt_old.z)) {
          continue;
        }

        Eigen::Vector3f p_old(pt_old.x, pt_old.y, pt_old.z);
        Eigen::Vector3f p_now = T_base_now_base_old * p_old;

        const float x = p_now.x();
        const float y = p_now.y();
        const float z = p_now.z();

        const double range_xy = std::hypot(static_cast<double>(x), static_cast<double>(y));

        // 近处不融合历史帧，防止拖影
        if (!is_current_frame && range_xy < near_range_no_fusion_) {
          continue;
        }

        pcl::PointXYZ pt_new;
        pt_new.x = x;
        pt_new.y = y;
        pt_new.z = z;
        fused->points.push_back(pt_new);
      }
    }

    fused->width = fused->points.size();
    fused->height = 1;
    fused->is_dense = false;
    return fused;
  }

  // ============================================================
  // 函数：障碍膨胀
  // 用途：给障碍物加安全边界，让后续规划更稳
  // ============================================================
  void inflateCosts(std::vector<int> & cost_data, const std::vector<bool> & obstacle_mask)
  {
    if (inflate_radius_ <= 1e-6) {
      return;
    }

    const int radius_cells = static_cast<int>(std::ceil(inflate_radius_ / resolution_));
    std::vector<int> inflated = cost_data;

    for (unsigned int my = 0; my < grid_height_; ++my) {
      for (unsigned int mx = 0; mx < grid_width_; ++mx) {
        const size_t idx = my * grid_width_ + mx;
        if (!obstacle_mask[idx]) {
          continue;
        }

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int nx = static_cast<int>(mx) + dx;
            const int ny = static_cast<int>(my) + dy;

            if (nx < 0 || ny < 0 ||
              nx >= static_cast<int>(grid_width_) ||
              ny >= static_cast<int>(grid_height_))
            {
              continue;
            }

            const double dist = std::hypot(
              static_cast<double>(dx) * resolution_,
              static_cast<double>(dy) * resolution_);

            if (dist > inflate_radius_) {
              continue;
            }

            int value = static_cast<int>(
              std::round(
                static_cast<double>(obstacle_occ_value_) *
                (1.0 - dist / inflate_radius_)));

            value = std::max(value, rough_occ_value_);
            value = std::clamp(value, 0, 100);

            const size_t nidx =
              static_cast<size_t>(ny) * grid_width_ + static_cast<size_t>(nx);
            inflated[nidx] = std::max(inflated[nidx], value);
          }
        }
      }
    }

    cost_data.swap(inflated);
  }

  // ============================================================
  // 函数：发布 OccupancyGrid
  // 用途：把生成好的 raw/cost 栅格图发出去
  // ============================================================
  void publishGrid(
    const rclcpp::Time & stamp,
    const std::vector<int8_t> & raw_data,
    const std::vector<int> & cost_data)
  {
    nav_msgs::msg::OccupancyGrid raw_msg;
    raw_msg.header.stamp = stamp;
    raw_msg.header.frame_id = target_frame_;
    raw_msg.info.resolution = resolution_;
    raw_msg.info.width = grid_width_;
    raw_msg.info.height = grid_height_;
    raw_msg.info.origin.position.x = x_min_;
    raw_msg.info.origin.position.y = y_min_;
    raw_msg.info.origin.position.z = 0.0;
    raw_msg.info.origin.orientation.w = 1.0;
    raw_msg.data = raw_data;

    nav_msgs::msg::OccupancyGrid cost_msg;
    cost_msg.header = raw_msg.header;
    cost_msg.info = raw_msg.info;
    cost_msg.data.resize(cost_data.size());

    for (size_t i = 0; i < cost_data.size(); ++i) {
      cost_msg.data[i] = static_cast<int8_t>(std::clamp(cost_data[i], 0, 100));
    }

    raw_pub_->publish(raw_msg);
    cost_pub_->publish(cost_msg);
  }

  // ============================================================
  // 函数：点云回调
  // 用途：
  //   1. 当前点云转到当前 base_link
  //   2. 查询当前 base_link 在 odom 下位姿
  //   3. 放入缓存
  //   4. 做多帧融合（可选）
  //   5. 栅格化并发布
  // ============================================================
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const rclcpp::Time msg_stamp(msg->header.stamp);

    // ------------------------------------------------
    // 1) 查当前点云到 base_link 的 TF
    // ------------------------------------------------
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
        target_frame_,
        msg->header.frame_id,
        msg_stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "TF lookup failed (%s -> %s): %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    // ------------------------------------------------
    // 2) 读取原始点云
    // ------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // ------------------------------------------------
    // 3) 构造当前帧：从原始点云坐标系 -> 当前 base_link
    // ------------------------------------------------
    Eigen::Translation3f translation(
      static_cast<float>(tf_msg.transform.translation.x),
      static_cast<float>(tf_msg.transform.translation.y),
      static_cast<float>(tf_msg.transform.translation.z));

    Eigen::Quaternionf q(
      static_cast<float>(tf_msg.transform.rotation.w),
      static_cast<float>(tf_msg.transform.rotation.x),
      static_cast<float>(tf_msg.transform.rotation.y),
      static_cast<float>(tf_msg.transform.rotation.z));

    Eigen::Affine3f T = translation * q;

    auto cloud_now_in_base =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_now_in_base->points.reserve(cloud.points.size());

    for (const auto & pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }

      Eigen::Vector3f p_in(pt.x, pt.y, pt.z);
      Eigen::Vector3f p = T * p_in;

      pcl::PointXYZ pout;
      pout.x = p.x();
      pout.y = p.y();
      pout.z = p.z();
      cloud_now_in_base->points.push_back(pout);
    }

    cloud_now_in_base->width = cloud_now_in_base->points.size();
    cloud_now_in_base->height = 1;
    cloud_now_in_base->is_dense = false;

    // ------------------------------------------------
    // 4) 如果开启多帧融合，查询 odom 位姿并缓存
    // ------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_bev = cloud_now_in_base;

    if (enable_temporal_fusion_) {
      geometry_msgs::msg::Pose current_pose_in_odom;
      if (lookupBasePoseInOdom(msg_stamp, current_pose_in_odom)) {
        TimedCloud item;
        item.cloud_in_base = cloud_now_in_base;
        item.pose_in_odom = current_pose_in_odom;
        item.stamp = msg_stamp;

        cloud_buffer_.push_back(item);
        pruneCloudBuffer(msg_stamp);

        cloud_for_bev = buildFusedCloud(current_pose_in_odom);
      } else {
        // 查不到 odom 位姿时，退化为单帧
        cloud_for_bev = cloud_now_in_base;
      }
    }

    // ------------------------------------------------
    // 5) 栅格统计
    // ------------------------------------------------
    std::vector<CellStat> cells(grid_width_ * grid_height_);

    for (const auto & pt : cloud_for_bev->points) {
      const double x = static_cast<double>(pt.x);
      const double y = static_cast<double>(pt.y);
      const double z = static_cast<double>(pt.z);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (x < x_min_ || x >= x_max_) {continue;}
      if (y < y_min_ || y >= y_max_) {continue;}
      if (z < z_min_ || z >= z_max_) {continue;}

      const unsigned int mx = static_cast<unsigned int>((x - x_min_) / resolution_);
      const unsigned int my = static_cast<unsigned int>((y - y_min_) / resolution_);

      if (mx >= grid_width_ || my >= grid_height_) {
        continue;
      }

      const size_t idx = my * grid_width_ + mx;
      auto & cell = cells[idx];
      cell.count += 1;
      cell.z_min = std::min(cell.z_min, static_cast<float>(z));
      cell.z_max = std::max(cell.z_max, static_cast<float>(z));
    }

    // ------------------------------------------------
    // 6) 根据每个格子的统计信息生成 OccupancyGrid
    // ------------------------------------------------
    std::vector<int8_t> raw_data(grid_width_ * grid_height_, -1);
    std::vector<int> cost_data(grid_width_ * grid_height_, unknown_occ_value_);
    std::vector<bool> obstacle_mask(grid_width_ * grid_height_, false);

    for (size_t idx = 0; idx < cells.size(); ++idx) {
      const auto & cell = cells[idx];

      if (cell.count < min_points_per_cell_) {
        raw_data[idx] = -1;
        cost_data[idx] = unknown_occ_value_;
        continue;
      }

      const double dz = static_cast<double>(cell.z_max - cell.z_min);
      const double z_max = static_cast<double>(cell.z_max);

      // 平坦区域：可通行
      if (dz < flat_height_diff_thresh_ && z_max < flat_z_max_thresh_) {
        raw_data[idx] = 0;
        cost_data[idx] = free_occ_value_;
      }
      // 明显突起：障碍
      else if (dz > obstacle_height_diff_thresh_ || z_max > obstacle_z_max_thresh_) {
        raw_data[idx] = 100;
        cost_data[idx] = obstacle_occ_value_;
        obstacle_mask[idx] = true;
      }
      // 中间状态：谨慎通过区域
      else {
        raw_data[idx] = 50;
        cost_data[idx] = rough_occ_value_;
      }
    }

    // ------------------------------------------------
    // 7) 膨胀障碍，形成安全边界
    // ------------------------------------------------
    inflateCosts(cost_data, obstacle_mask);

    // ------------------------------------------------
    // 8) 发布结果
    // ------------------------------------------------
    publishGrid(msg_stamp, raw_data, cost_data);
  }

  // ============================================================
  // 成员变量
  // ============================================================

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // topic / frame
  std::string cloud_topic_;
  std::string target_frame_;
  std::string odom_frame_;
  std::string publish_raw_topic_;
  std::string publish_cost_topic_;

  // TF
  double tf_timeout_sec_;

  // BEV 范围与分辨率
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double resolution_;
  double z_min_;
  double z_max_;

  // 栅格判定
  int min_points_per_cell_;
  double flat_height_diff_thresh_;
  double flat_z_max_thresh_;
  double obstacle_height_diff_thresh_;
  double obstacle_z_max_thresh_;

  // OccupancyGrid 代价值
  int unknown_occ_value_;
  int rough_occ_value_;
  int obstacle_occ_value_;
  int free_occ_value_;

  // 膨胀半径
  double inflate_radius_;

  // 地图尺寸
  unsigned int grid_width_;
  unsigned int grid_height_;

  // 多帧融合参数
  bool enable_temporal_fusion_;
  int fusion_frame_count_;
  double fusion_time_window_;
  double near_range_no_fusion_;

  // 多帧缓存
  std::deque<TimedCloud> cloud_buffer_;
};

// 主函数：启动节点
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BevLocalMapNode>());
  rclcpp::shutdown();
  return 0;
}

