// kiss_icp_ros/include/kiss_icp_ros/kiss_icp_node.hpp
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "kiss_icp_core/kiss_icp.h"

namespace kiss_icp_ros {

class KissICPNode : public rclcpp::Node {
 public:
  explicit KissICPNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~KissICPNode();

  nav_msgs::msg::Odometry getCurrentOdometry() const;
  void resetOdometry();

 private:
  // Core components
  std::unique_ptr<KissICP> kiss_icp_;

  // ROS publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_;

  // ROS subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // TF2 components
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Frame IDs
  std::string base_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;

  // State variables
  mutable std::mutex state_mutex_;
  std::atomic<bool> initialized_{false};
  nav_msgs::msg::Path path_msg_;
  nav_msgs::msg::Odometry current_odom_;
  rclcpp::Time last_scan_time_;
  size_t processed_scans_{0};

  // Core functions
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void declareParameters();
  void initializePublishers();
  void initializeSubscribers();
  void initializeTF();

  // Conversion functions
  std::pair<PointCloud, std::vector<float>> convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  sensor_msgs::msg::PointCloud2 convertToROS(const PointCloud& points, const std_msgs::msg::Header& header);

  // Publishing functions
  void publishOdometry(const OdometryResult& result, const std_msgs::msg::Header& header);
  void publishPose(const OdometryResult& result, const std_msgs::msg::Header& header);
  void publishPath(const OdometryResult& result, const std_msgs::msg::Header& header);
  void publishTransforms(const OdometryResult& result, const std_msgs::msg::Header& header);
  void publishLocalMap(const OdometryResult& result, const std_msgs::msg::Header& header);
  void publishCurrentScan(const PointCloud& points, const std_msgs::msg::Header& header);

  // Utility functions
  geometry_msgs::msg::Pose eigenToPose(const Eigen::Matrix4f& matrix);
  geometry_msgs::msg::Transform eigenToTransform(const Eigen::Matrix4f& matrix);
  bool validatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

}  // namespace kiss_icp_ros