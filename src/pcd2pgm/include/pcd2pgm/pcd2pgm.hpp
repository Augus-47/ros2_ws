#include <pcl/filters/passthrough.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
// 解除注释：启用平面分割相关头文件（必须！）
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// 引入PCL点云类型定义（避免类型未定义错误）
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcd2pgm
{
class PCLFiltersNode : public rclcpp::Node
{
private:
  float thre_z_min_;
  float thre_z_max_;
  float thre_radius_;
  bool flag_pass_through_;
  float map_resolution_;
  int thres_point_count_;
  std::string pcd_file_;
  std::string map_topic_name_;
  std::string file_directory_, file_name_;

  // 新增：点云裁剪范围（空旷场景用，从参数读取）
  double crop_range_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius_;
  nav_msgs::msg::OccupancyGrid map_topic_msg_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr time_;

  void declareParameters();

  void getParameters();

  // 直通滤波（保留接口，未使用）
  void passThroughFilter(const double & thre_low, const double & thre_high, const bool & flag_in);

  // 半径滤波（优化为统计+半径双滤波）
  void radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud0, const double & radius,
    const int & thre_count);

  // 新增：平面分割+高度约束函数（空旷场景核心，必须声明）
  void segmentGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud);

  void setMapTopicMsg(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg);

  void publishMap();

public:
  explicit PCLFiltersNode(const rclcpp::NodeOptions & options);
  ~PCLFiltersNode();
};
}  // namespace pcd2pgm

