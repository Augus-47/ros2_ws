#include <lidar_localization/lidar_localization_component.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

// tf2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);

  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);

  declare_parameter("reject_translation_threshold", 0.5);
  declare_parameter("reject_yaw_threshold_deg", 5.0);
  declare_parameter("reject_z_threshold", 0.3);
  declare_parameter("reject_roll_pitch_threshold_deg", 10.0);

  declare_parameter("localization_success_frame_num", 3);
  declare_parameter("init_score_threshold", 3.0);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  localized_ = false;
  consecutive_success_count_ = 0;
  odom_initialized_ = false;

  pose_pub_->on_activate();
  orin_pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = msg->header.stamp;
    pose_stamped.header.frame_id = global_frame_id_;
    pose_stamped.pose = msg->pose.pose;
    path_ptr_->poses.push_back(pose_stamped);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud_ptr, map_msg);
    map_msg.header.frame_id = global_frame_id_;
    initial_map_pub_->publish(map_msg);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();
  orin_pose_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  orin_pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);

  get_parameter("reject_translation_threshold", reject_translation_threshold_);
  get_parameter("reject_yaw_threshold_deg", reject_yaw_threshold_deg_);
  get_parameter("reject_z_threshold", reject_z_threshold_);
  get_parameter("reject_roll_pitch_threshold_deg", reject_roll_pitch_threshold_deg_);

  get_parameter("localization_success_frame_num", localization_success_frame_num_);
  get_parameter("init_score_threshold", init_score_threshold_);
}

void PCLLocalization::initializePubSub()
{
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  orin_pose_pub_ = create_publisher<car_msg::msg::OrinPose>(
    "orin_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));
}

void PCLLocalization::initializeRegistration()
{
  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    ndt_omp->setNumThreads(ndt_num_threads_ > 0 ? ndt_num_threads_ : omp_get_max_threads());
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }

  registration_->setMaximumIterations(ndt_max_iterations_);
  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_ || !msg) { return; }

  tf2::Transform T_odom_base;
  tf2::fromMsg(msg->pose.pose, T_odom_base);

  if (!odom_initialized_) {
    last_odom_pose_ = T_odom_base;
    current_odom_pose_ = T_odom_base;
    odom_initialized_ = true;

    if (enable_debug_) {
      RCLCPP_INFO(get_logger(), "[odom] initialized.");
    }
    return;
  }

  last_odom_pose_ = current_odom_pose_;
  current_odom_pose_ = T_odom_base;
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) { return; }
  // keep your original logic
}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (use_odom_ && !odom_initialized_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "use_odom is true, but odom has not been initialized yet.");
  }

  if (!msg) { return; }
  if (!map_recieved_ || !initialpose_recieved_) { return; }

  const std::string cloud_frame = msg->header.frame_id;
  const rclcpp::Time stamp(msg->header.stamp);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  if (use_imu_) {
    double received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI> tmp;
  tmp.reserve(filtered_cloud_ptr->size());
  for (const auto & p : filtered_cloud_ptr->points) {
    double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);

  // prediction
  tf2::Transform T_map_base_last;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, T_map_base_last);

  tf2::Transform T_pred = T_map_base_last;

  if (use_odom_ && odom_initialized_) {
    tf2::Transform delta_odom = last_odom_pose_.inverse() * current_odom_pose_;
    T_pred = T_map_base_last * delta_odom;

    if (enable_debug_) {
      double roll, pitch, yaw;
      tf2::Matrix3x3(T_pred.getRotation()).getRPY(roll, pitch, yaw);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[predict] use odom prior. pred=(%.3f, %.3f, %.3f), yaw=%.3f deg",
        T_pred.getOrigin().x(),
        T_pred.getOrigin().y(),
        T_pred.getOrigin().z(),
        yaw * 180.0 / M_PI);
    }
  } else {
    if (enable_debug_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[predict] odom prior not available, fallback to last matched pose.");
    }
  }

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  tf2::Matrix3x3 rot_tf = T_pred.getBasis();
  tf2::Vector3 trans_tf = T_pred.getOrigin();

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      init_guess(r, c) = static_cast<float>(rot_tf[r][c]);
    }
  }
  init_guess(0, 3) = static_cast<float>(trans_tf.x());
  init_guess(1, 3) = static_cast<float>(trans_tf.y());
  init_guess(2, 3) = static_cast<float>(trans_tf.z());

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  registration_->align(*output_cloud, init_guess);

  if (!registration_->hasConverged()) {
    consecutive_success_count_ = 0;
    RCLCPP_WARN(get_logger(), "[%s] The registration didn't converge.",
      localized_ ? "TRACKING" : "INITIALIZING");
    return;
  }

  const double fitness_score = registration_->getFitnessScore();
  const double active_score_threshold = localized_ ? score_threshold_ : init_score_threshold_;

  if (fitness_score > active_score_threshold) {
    consecutive_success_count_ = 0;
    RCLCPP_WARN(
      get_logger(),
      "[%s] Reject pose: fitness score is over %lf (th=%lf).",
      localized_ ? "TRACKING" : "INITIALIZING",
      fitness_score, active_score_threshold);
    return;
  }

  Eigen::Matrix4f T_map_cloud_e = registration_->getFinalTransformation();
  Eigen::Matrix3d rot_mat = T_map_cloud_e.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond q_map_cloud(rot_mat);

  tf2::Transform T_map_cloud;
  T_map_cloud.setOrigin(tf2::Vector3(
    static_cast<double>(T_map_cloud_e(0, 3)),
    static_cast<double>(T_map_cloud_e(1, 3)),
    static_cast<double>(T_map_cloud_e(2, 3))
  ));
  T_map_cloud.setRotation(tf2::Quaternion(
    q_map_cloud.x(), q_map_cloud.y(), q_map_cloud.z(), q_map_cloud.w()
  ));

  tf2::Transform T_cloud_base;
  T_cloud_base.setIdentity();

  if (cloud_frame != base_frame_id_) {
    try {
      auto tf_cloud_base_msg = tfbuffer_.lookupTransform(
        cloud_frame, base_frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf_cloud_base_msg.transform, T_cloud_base);
    } catch (const tf2::TransformException & ex) {
      consecutive_success_count_ = 0;
      RCLCPP_WARN(get_logger(),
                  "No TF %s -> %s, cannot convert map->cloud to map->base. (%s)",
                  cloud_frame.c_str(), base_frame_id_.c_str(), ex.what());
      return;
    }
  }

  tf2::Transform T_map_base = T_map_cloud * T_cloud_base;

  // jump gate only in tracking mode
  if (localized_) {
    tf2::Transform T_diff = T_pred.inverse() * T_map_base;

    const double dx = T_diff.getOrigin().x();
    const double dy = T_diff.getOrigin().y();
    const double dz = T_diff.getOrigin().z();
    const double dtrans = std::sqrt(dx * dx + dy * dy);

    double droll, dpitch, dyaw;
    tf2::Matrix3x3(T_diff.getRotation()).getRPY(droll, dpitch, dyaw);

    const double dyaw_deg = std::fabs(dyaw) * 180.0 / M_PI;
    const double droll_deg = std::fabs(droll) * 180.0 / M_PI;
    const double dpitch_deg = std::fabs(dpitch) * 180.0 / M_PI;

    if (dtrans > reject_translation_threshold_ ||
        std::fabs(dz) > reject_z_threshold_ ||
        dyaw_deg > reject_yaw_threshold_deg_ ||
        droll_deg > reject_roll_pitch_threshold_deg_ ||
        dpitch_deg > reject_roll_pitch_threshold_deg_) {
      consecutive_success_count_ = 0;
      RCLCPP_WARN(
        get_logger(),
        "[TRACKING] Reject pose: jump too large. dtrans=%.3f m dz=%.3f m dyaw=%.3f deg droll=%.3f deg dpitch=%.3f deg",
        dtrans, dz, dyaw_deg, droll_deg, dpitch_deg);
      return;
    }
  }

  tf2::Transform T_odom_base;
  try {
    auto tf_odom_base_msg = tfbuffer_.lookupTransform(
      odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
    tf2::fromMsg(tf_odom_base_msg.transform, T_odom_base);
  } catch (const tf2::TransformException & ex1) {
    try {
      auto tf_odom_base_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf_odom_base_msg.transform, T_odom_base);
      RCLCPP_WARN(get_logger(), "lookupTransform(odom->%s) fallback TimePointZero: %s",
                  base_frame_id_.c_str(), ex1.what());
    } catch (const tf2::TransformException & ex2) {
      consecutive_success_count_ = 0;
      RCLCPP_WARN(get_logger(), "TF lookup failed (odom->%s): %s",
                  base_frame_id_.c_str(), ex2.what());
      return;
    }
  }

  tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

  // switch mode
  if (!localized_) {
    consecutive_success_count_++;

    RCLCPP_INFO(
      get_logger(),
      "[INITIALIZING] success %d / %d, fitness=%.4f",
      consecutive_success_count_,
      localization_success_frame_num_,
      fitness_score);

    if (consecutive_success_count_ >= localization_success_frame_num_) {
      localized_ = true;
      RCLCPP_INFO(get_logger(), "Localization locked. Switch to TRACKING mode.");
    }
  }

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = msg->header.stamp;
  map_to_odom.header.frame_id = global_frame_id_;
  map_to_odom.child_frame_id = odom_frame_id_;
  map_to_odom.transform = tf2::toMsg(T_map_odom);
  broadcaster_.sendTransform(map_to_odom);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg = *corrent_pose_with_cov_stamped_ptr_;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = global_frame_id_;
  pose_msg.pose.pose.position.x = T_map_base.getOrigin().x();
  pose_msg.pose.pose.position.y = T_map_base.getOrigin().y();
  pose_msg.pose.pose.position.z = T_map_base.getOrigin().z();
  pose_msg.pose.pose.orientation = tf2::toMsg(T_map_base.getRotation());
  corrent_pose_with_cov_stamped_ptr_ =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_msg);
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  car_msg::msg::OrinPose current_orin_pose;
  current_orin_pose.header.stamp = msg->header.stamp;
  current_orin_pose.header.frame_id = global_frame_id_;
  current_orin_pose.pose = pose_msg.pose.pose;
  orin_pose_pub_->publish(current_orin_pose);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = msg->header.stamp;
  pose_stamped.header.frame_id = global_frame_id_;
  pose_stamped.pose = pose_msg.pose.pose;
  path_ptr_->poses.push_back(pose_stamped);
  path_pub_->publish(*path_ptr_);

  last_scan_ptr_ = msg;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                       "Publish TF: %s->%s (map->odom). cloud_frame=%s base=%s mode=%s",
                       global_frame_id_.c_str(), odom_frame_id_.c_str(),
                       cloud_frame.c_str(), base_frame_id_.c_str(),
                       localized_ ? "TRACKING" : "INITIALIZING");
}

