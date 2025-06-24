#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "kiss_icp_core/kiss_icp.h"

namespace kiss_icp_ros {

/**
 * @brief ROS 2 node for KISS-ICP LiDAR odometry
 *
 * This node provides a ROS 2 interface for the KISS-ICP algorithm.
 * It subscribes to LiDAR point clouds and publishes odometry estimates,
 * poses, and visualizations.
 */
class KissICPNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   */
  KissICPNode();

  /**
   * @brief Destructor
   */
  ~KissICPNode();

 private:
  // Core KISS-ICP instance - FIXED: removed namespace
  std::unique_ptr<KissICP> kiss_icp_;

  // ROS 2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // ROS 2 publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub_;

  // TF2
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // State variables
  nav_msgs::msg::Path path_msg_;
  bool first_cloud_received_;
  rclcpp::Time last_cloud_time_;

  // Frame IDs
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  // Parameters
  double max_range_;
  double min_deviation_;
  double voxel_size_;
  int max_iterations_;
  double convergence_threshold_;
  bool publish_tf_;
  bool publish_map_;
  bool publish_deskewed_;
  double map_publish_rate_;
  std::string cloud_topic_;

  // Timers
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  /**
   * @brief Declare and load ROS parameters
   */
  void declareParameters();

  /**
   * @brief Point cloud callback
   * @param msg Point cloud message
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Publish odometry message
   * @param pose Current pose
   * @param header Message header
   */
  void publishOdometry(const Eigen::Matrix4f& pose, const std_msgs::msg::Header& header);

  /**
   * @brief Publish pose message
   * @param pose Current pose
   * @param header Message header
   */
  void publishPose(const Eigen::Matrix4f& pose, const std_msgs::msg::Header& header);

  /**
   * @brief Update and publish path
   * @param pose Current pose
   * @param header Message header
   */
  void publishPath(const Eigen::Matrix4f& pose, const std_msgs::msg::Header& header);

  /**
   * @brief Publish TF transform
   * @param pose Current pose
   * @param header Message header
   */
  void publishTransform(const Eigen::Matrix4f& pose, const std_msgs::msg::Header& header);

  /**
   * @brief Timer callback to publish map
   */
  void mapPublishCallback();

  /**
   * @brief Publish local map as point cloud
   */
  void publishMap();

  /**
   * @brief Publish deskewed point cloud for debugging
   * @param cloud Deskewed point cloud
   * @param header Message header
   */
  void publishDeskewedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std_msgs::msg::Header& header);

  /**
   * @brief Convert Eigen matrix to geometry_msgs pose
   * @param matrix 4x4 transformation matrix
   * @return Pose message
   */
  geometry_msgs::msg::Pose eigenToPose(const Eigen::Matrix4f& matrix);

  /**
   * @brief Convert Eigen matrix to geometry_msgs transform
   * @param matrix 4x4 transformation matrix
   * @return Transform message
   */
  geometry_msgs::msg::Transform eigenToTransform(const Eigen::Matrix4f& matrix);

  /**
   * @brief Extract timestamps from point cloud
   * @param cloud Input point cloud
   * @return Vector of normalized timestamps [0,1]
   */
  std::vector<float> extractTimestamps(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

}  // namespace kiss_icp_ros