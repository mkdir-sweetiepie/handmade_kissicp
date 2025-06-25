// kiss_icp_ros/src/kiss_icp_node.cpp
#include "kiss_icp_ros/kiss_icp_node.hpp"

namespace kiss_icp_ros {

KissICPNode::KissICPNode(const rclcpp::NodeOptions& options) : Node("kiss_icp_node", options) {
  RCLCPP_INFO(get_logger(), "Starting KISS-ICP Node");

  declareParameters();
  initializePublishers();
  initializeSubscribers();
  initializeTF();

  // Initialize KISS-ICP
  kiss_icp_ = std::make_unique<KissICP>();

  // Configure KISS-ICP with parameters
  double max_range = get_parameter("max_range").as_double();
  double scan_duration = get_parameter("scan_duration").as_double();

  kiss_icp_->setMaxRange(max_range);
  kiss_icp_->setScanDuration(scan_duration);

  // Initialize path message
  path_msg_.header.frame_id = odom_frame_;

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "KISS-ICP Node initialized successfully");
}

KissICPNode::~KissICPNode() { RCLCPP_INFO(get_logger(), "Shutting down KISS-ICP Node"); }

nav_msgs::msg::Odometry KissICPNode::getCurrentOdometry() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_odom_;
}

void KissICPNode::resetOdometry() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (kiss_icp_) {
    kiss_icp_->reset();
  }
  path_msg_.poses.clear();
  processed_scans_ = 0;
  RCLCPP_INFO(get_logger(), "Odometry reset completed");
}

void KissICPNode::declareParameters() {
  // Frame parameters
  declare_parameter("base_frame", "base_link");
  declare_parameter("odom_frame", "odom");
  declare_parameter("lidar_frame", "velodyne");

  // Algorithm parameters
  declare_parameter("max_range", 100.0);
  declare_parameter("min_range", 1.0);
  declare_parameter("scan_duration", 0.1);
  declare_parameter("voxel_size", 0.5);

  // Publishing parameters
  declare_parameter("publish_tf", true);
  declare_parameter("publish_path", true);
  declare_parameter("publish_local_map", true);

  // Get frame IDs
  base_frame_ = get_parameter("base_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();

  RCLCPP_INFO(get_logger(), "Parameters loaded:");
  RCLCPP_INFO(get_logger(), "  base_frame: %s", base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  odom_frame: %s", odom_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  lidar_frame: %s", lidar_frame_.c_str());
}

void KissICPNode::initializePublishers() {
  // Create publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
  local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 1);
  current_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", 1);

  RCLCPP_INFO(get_logger(), "Publishers initialized");
}

void KissICPNode::initializeSubscribers() {
  // Create subscriber
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", rclcpp::SensorDataQoS(), std::bind(&KissICPNode::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribers initialized");
}

void KissICPNode::initializeTF() {
  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "TF broadcaster initialized");
}

void KissICPNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!initialized_) {
    RCLCPP_WARN(get_logger(), "Node not yet initialized, skipping scan");
    return;
  }

  if (!validatePointCloud(msg)) {
    RCLCPP_WARN(get_logger(), "Invalid point cloud received");
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  try {
    // Convert ROS message to internal format
    auto [points, timestamps] = convertPointCloud(msg);

    if (points.empty()) {
      RCLCPP_WARN(get_logger(), "Empty point cloud after conversion");
      return;
    }

    // Process with KISS-ICP
    auto result = kiss_icp_->processFrame(points, timestamps);

    if (!result.success) {
      RCLCPP_WARN(get_logger(), "KISS-ICP processing failed");
      return;
    }

    // Publish results
    publishOdometry(result, msg->header);
    publishPose(result, msg->header);

    if (get_parameter("publish_path").as_bool()) {
      publishPath(result, msg->header);
    }

    if (get_parameter("publish_tf").as_bool()) {
      publishTransforms(result, msg->header);
    }

    if (get_parameter("publish_local_map").as_bool()) {
      publishLocalMap(result, msg->header);
    }

    publishCurrentScan(points, msg->header);

    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    processed_scans_++;
    last_scan_time_ = msg->header.stamp;

    RCLCPP_DEBUG(get_logger(), "Processed scan %zu in %ld ms (%zu points)", processed_scans_, duration.count(), points.size());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error processing point cloud: %s", e.what());
  }
}

std::pair<PointCloud, std::vector<float>> KissICPNode::convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  PointCloud points;
  std::vector<float> timestamps;

  // Convert using PCL
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);

  points.reserve(pcl_cloud.size());
  timestamps.reserve(pcl_cloud.size());

  // Convert each point
  for (const auto& pcl_point : pcl_cloud) {
    if (std::isfinite(pcl_point.x) && std::isfinite(pcl_point.y) && std::isfinite(pcl_point.z)) {
      Point3D point;
      point.x = pcl_point.x;
      point.y = pcl_point.y;
      point.z = pcl_point.z;
      points.push_back(point);
      timestamps.push_back(0.0f);  // Simplified timestamp
    }
  }

  return {points, timestamps};
}

sensor_msgs::msg::PointCloud2 KissICPNode::convertToROS(const PointCloud& points, const std_msgs::msg::Header& header) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.reserve(points.size());

  for (const auto& point : points) {
    pcl_cloud.emplace_back(point.x, point.y, point.z);
  }

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ros_cloud.header = header;

  return ros_cloud;
}

void KissICPNode::publishOdometry(const OdometryResult& result, const std_msgs::msg::Header& header) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = header;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  // Set pose
  odom_msg.pose.pose = eigenToPose(result.pose);

  // Set velocity (simplified)
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  // Set covariance matrices
  std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
  std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);

  // Diagonal covariance values
  odom_msg.pose.covariance[0] = 0.1;   // x
  odom_msg.pose.covariance[7] = 0.1;   // y
  odom_msg.pose.covariance[35] = 0.1;  // yaw

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_odom_ = odom_msg;
  }

  odom_pub_->publish(odom_msg);
}

void KissICPNode::publishPose(const OdometryResult& result, const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = header;
  pose_msg.header.frame_id = odom_frame_;
  pose_msg.pose = eigenToPose(result.pose);

  pose_pub_->publish(pose_msg);
}

void KissICPNode::publishPath(const OdometryResult& result, const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.header.frame_id = odom_frame_;
  pose_stamped.pose = eigenToPose(result.pose);

  path_msg_.header = header;
  path_msg_.header.frame_id = odom_frame_;
  path_msg_.poses.push_back(pose_stamped);

  // Limit path size
  const size_t max_path_size = 1000;
  if (path_msg_.poses.size() > max_path_size) {
    path_msg_.poses.erase(path_msg_.poses.begin());
  }

  path_pub_->publish(path_msg_);
}

void KissICPNode::publishTransforms(const OdometryResult& result, const std_msgs::msg::Header& header) {
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header = header;
  transform_msg.header.frame_id = odom_frame_;
  transform_msg.child_frame_id = base_frame_;
  transform_msg.transform = eigenToTransform(result.pose);

  tf_broadcaster_->sendTransform(transform_msg);
}

void KissICPNode::publishLocalMap(const OdometryResult& result, const std_msgs::msg::Header& header) {
  if (!result.local_map_points.empty()) {
    auto map_msg = convertToROS(result.local_map_points, header);
    map_msg.header.frame_id = odom_frame_;
    local_map_pub_->publish(map_msg);
  }
}

void KissICPNode::publishCurrentScan(const PointCloud& points, const std_msgs::msg::Header& header) {
  auto scan_msg = convertToROS(points, header);
  current_scan_pub_->publish(scan_msg);
}

geometry_msgs::msg::Pose KissICPNode::eigenToPose(const Eigen::Matrix4f& matrix) {
  geometry_msgs::msg::Pose pose;

  // Translation
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);

  // Rotation (convert rotation matrix to quaternion)
  Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
  Eigen::Quaternionf quat(rotation);

  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  return pose;
}

geometry_msgs::msg::Transform KissICPNode::eigenToTransform(const Eigen::Matrix4f& matrix) {
  geometry_msgs::msg::Transform transform;

  // Translation
  transform.translation.x = matrix(0, 3);
  transform.translation.y = matrix(1, 3);
  transform.translation.z = matrix(2, 3);

  // Rotation
  Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
  Eigen::Quaternionf quat(rotation);

  transform.rotation.x = quat.x();
  transform.rotation.y = quat.y();
  transform.rotation.z = quat.z();
  transform.rotation.w = quat.w();

  return transform;
}

bool KissICPNode::validatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!msg) {
    RCLCPP_ERROR(get_logger(), "Null point cloud message");
    return false;
  }

  if (msg->data.empty()) {
    RCLCPP_WARN(get_logger(), "Empty point cloud data");
    return false;
  }

  if (msg->width == 0 || msg->height == 0) {
    RCLCPP_WARN(get_logger(), "Invalid point cloud dimensions");
    return false;
  }

  return true;
}

}  // namespace kiss_icp_ros