#include "pcd2pgm/pcd2pgm.hpp"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <rclcpp/qos.hpp>
#include <algorithm>
#include <limits>

namespace pcd2pgm
{
PCLFiltersNode::PCLFiltersNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud->points.size());

  // 1. 平面分割+高度约束：彻底剔除地面及地面噪声（空旷场景核心）
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  segmentGroundPlane(pcd_cloud, cloud_no_ground);
  if (cloud_no_ground->empty()) {
    RCLCPP_WARN(get_logger(), "No obstacle points left after ground segmentation!");
    return;
  }

  // 2. 统计+半径双滤波：适配稀疏障碍物，过滤孤立噪声
  radiusOutlierFilter(cloud_no_ground, thre_radius_, thres_point_count_);
  if (cloud_after_Radius_->empty()) {
    RCLCPP_WARN(get_logger(), "No points left after outlier filtering!");
    return;
  }

  // 3. 动态分辨率+范围裁剪：生成适配空旷场景的地图
  setMapTopicMsg(cloud_after_Radius_, map_topic_msg_);

  time_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PCLFiltersNode::publishMap, this));
  RCLCPP_INFO(get_logger(), "Node initialized successfully (outdoor empty scene mode)");
}

PCLFiltersNode::~PCLFiltersNode() {}

void PCLFiltersNode::publishMap()
{
  if (rclcpp::ok()) {
    map_publisher_->publish(map_topic_msg_);
  }
}

void PCLFiltersNode::declareParameters()
{
  // 空旷室外场景优化参数
  declare_parameter("file_directory", "/home/lihanchen/Downloads/pcd2pgm/"); //config来改
  declare_parameter("file_name", "RMUC");
  declare_parameter("thre_z_min", 0.5);    // 高度约束下限（适配低矮障碍物）(改参数去config的yaml去改）
  declare_parameter("thre_z_max", 2.0);    // 高度约束上限（覆盖多数室外障碍物）2.4--1.4
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.3);   // 半径滤波搜索半径（宽松）
  declare_parameter("map_resolution", 0.05); // 基础分辨率（动态调整）
  declare_parameter("thres_point_count", 1); // 邻点阈值（适配稀疏点云）
  declare_parameter("map_topic_name", "map");
  declare_parameter("crop_range", 50.0);   // 点云裁剪范围（±50m）
}

void PCLFiltersNode::getParameters()
{
  get_parameter("file_directory", file_directory_);
  get_parameter("file_name", file_name_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("crop_range", crop_range_); // 新增裁剪范围参数
  pcd_file_ = file_directory_ + file_name_ + ".pcd";
}

// 平面分割+高度约束：彻底剔除地面（空旷场景适配）
void PCLFiltersNode::segmentGroundPlane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr ground_inliers = std::make_shared<pcl::PointIndices>();
  pcl::ModelCoefficients::Ptr plane_coeffs = std::make_shared<pcl::ModelCoefficients>();
  
  // --- 插入调试代码开始 ---
double min_z_found = 1000.0;
double max_z_found = -1000.0;
for (const auto & point : output_cloud->points) {
    if (point.z < min_z_found) min_z_found = point.z;
    if (point.z > max_z_found) max_z_found = point.z;
}
RCLCPP_ERROR(get_logger(), "!!! DEBUG CHECK !!! Cloud Z Range: [%.2f, %.2f]", min_z_found, max_z_found);
RCLCPP_ERROR(get_logger(), "!!! DEBUG CHECK !!! Using Filter:  [%.2f, %.2f]", thre_z_min_, thre_z_max_);
// --- 插入调试代码结束 ---

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 改为垂直于某轴的平面
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));         // 限定法向量为 Z 轴
  seg.setEpsAngle(0.15);                               // 允许的最大角度偏差（约8.5度）
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.15); 
  // 输入点云和执行分割
  seg.setInputCloud(input_cloud);
  seg.segment(*ground_inliers, *plane_coeffs);




  // 提取非地面点（障碍物）
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(ground_inliers);
  extract.setNegative(true);
  extract.filter(*output_cloud);

  // 叠加高度约束：过滤地面残留噪声和无效高度点
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_filtered =    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  
  for (const auto & point : output_cloud->points) {
    if (point.z >= thre_z_min_ && point.z <= thre_z_max_ &&
        !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
      cloud_height_filtered->points.push_back(point);
    }
  }
  cloud_height_filtered->width = cloud_height_filtered->points.size();
  cloud_height_filtered->height = 1;
  cloud_height_filtered->is_dense = false;

  output_cloud = cloud_height_filtered;

  RCLCPP_INFO(
    get_logger(), "Ground segmentation: input=%lu | removed_ground=%lu | remaining_obstacles=%lu",
    input_cloud->points.size(), ground_inliers->indices.size(), output_cloud->points.size());
}

// 直通滤波（保留接口，若需临时启用可取消注释）
void PCLFiltersNode::passThroughFilter(
  const double & thre_low, const double & thre_high, const bool & flag_in)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr capt = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_after_PassThrough_ = capt;

  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*cloud_after_PassThrough_);

  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering, point cloud size: %lu",
    cloud_after_PassThrough_->points.size());
}
  

// 统计+半径双滤波：适配空旷场景稀疏障碍物
void PCLFiltersNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud0, const double & radius,
  const int & thre_count)
{
  cloud_after_Radius_ = std::make_shared	<pcl::PointCloud<pcl::PointXYZ>>();

  // 优先使用统计滤波（更适合稀疏点云去噪）
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(pcd_cloud0);
  sor.setMeanK(30);                          // 考虑周围30个邻居
  sor.setStddevMulThresh(2.0);              // 标准差阈值（宽松）
  sor.filter(*cloud_after_Radius_);

  // 若统计滤波过于严格， fallback 到半径滤波（宽松参数）
  if (cloud_after_Radius_->size() < pcd_cloud0->size() * 0.3) {
    RCLCPP_WARN(get_logger(), "Statistical filter too strict, fallback to radius filter");
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(pcd_cloud0);
    radius_filter.setRadiusSearch(radius);
    radius_filter.setMinNeighborsInRadius(std::max(1, thre_count - 1)); // 邻点阈值-1
    radius_filter.filter(*cloud_after_Radius_);
  }

  RCLCPP_INFO(
    get_logger(), "After outlier filtering: input=%lu | output=%lu",
    pcd_cloud0->points.size(), cloud_after_Radius_->points.size());
}

// 动态分辨率+范围裁剪：生成适配空旷场景的地图
void PCLFiltersNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.info.map_load_time = this->now();

  // 1. 点云裁剪：限制有效范围（避免地图过大）
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & point : cloud->points) {
    if (std::fabs(point.x) <= crop_range_ && std::fabs(point.y) <= crop_range_ &&
        !std::isnan(point.x) && !std::isnan(point.y)) {
      cloud_cropped->points.push_back(point);
    }
  }
  if (cloud_cropped->empty()) {
    RCLCPP_WARN(get_logger(), "No valid points left after cropping!");
    return;
  }

  // 2. 计算裁剪后点云的XY范围
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();
  for (const auto & point : cloud_cropped->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }
  double x_range = x_max - x_min;
  double y_range = y_max - y_min;

  msg.info.resolution = map_resolution_;

  // 4. 计算地图尺寸
  msg.info.width = static_cast<unsigned int>(std::ceil(x_range / msg.info.resolution));
  msg.info.height = static_cast<unsigned int>(std::ceil(y_range / msg.info.resolution));
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  // 5. 初始化并填充地图数据
  msg.data.assign(msg.info.width * msg.info.height, 0);
  for (const auto & point : cloud_cropped->points) {
    int i = static_cast<int>(std::floor((point.x - x_min) / msg.info.resolution));
    int j = static_cast<int>(std::floor((point.y - y_min) / msg.info.resolution));

    // 双重边界检查，避免数组越界
    if (i >= 0 && i < static_cast<int>(msg.info.width) &&
        j >= 0 && j < static_cast<int>(msg.info.height)) {
      msg.data[i + j * msg.info.width] = 100; // 障碍物标记为100
    }
  }

  RCLCPP_INFO(
    get_logger(), "Map generated: %dx%d grids (res=%.3fm, range=%.2fm×%.2fm)",
    msg.info.width, msg.info.height, msg.info.resolution, x_range, y_range);
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::PCLFiltersNode)

