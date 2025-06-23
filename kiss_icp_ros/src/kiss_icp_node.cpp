// kiss_icp_ros/src/kiss_icp_node.cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "kiss_icp_core/kiss_icp.h"

class KissICPNode : public rclcpp::Node {
 private:
  KissICP odometry_;

  // Publishers & Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Frame IDs
  std::string odom_frame_;
  std::string base_frame_;

 public:
  KissICPNode() : Node("kiss_icp_node") {
    // Declare parameters
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("max_range", 100.0);
    declare_parameter("scan_duration", 0.1);

    // Get parameters
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    float max_range = get_parameter("max_range").as_double();
    float scan_duration = get_parameter("scan_duration").as_double();

    // Configure KISS-ICP
    odometry_.setMaxRange(max_range);
    odometry_.setScanDuration(scan_duration);

    // Initialize publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 1);
    scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);

    // Initialize subscriber
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 10, std::bind(&KissICPNode::pointCloudCallback, this, std::placeholders::_1));

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "KISS-ICP Node initialized");
    RCLCPP_INFO(get_logger(), "  odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  max_range: %.1f", max_range);
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Convert ROS message to internal format
    PointCloud points;
    std::vector<float> timestamps;
    convertFromROS(msg, points, timestamps);

    // Process frame
    auto result = odometry_.processFrame(points, timestamps);

    if (result.success) {
      publishOdometry(result, msg->header);
      publishLocalMap(result.local_map_points, msg->header);
      publishCurrentScan(result.current_scan, msg->header);
      publishPose(result, msg->header);
      publishTF(result, msg->header);

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

      RCLCPP_DEBUG(get_logger(), "Processed frame in %ld ms, ICP iterations: %d, threshold: %.3f", duration.count(), result.icp_iterations, result.adaptive_threshold);
    } else {
      RCLCPP_WARN(get_logger(), "Frame processing failed");
    }
  }

 private:
  void convertFromROS(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, PointCloud& points, std::vector<float>& timestamps) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    points.clear();
    points.reserve(pcl_cloud.points.size());

    for (const auto& pt : pcl_cloud.points) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        points.emplace_back(pt.x, pt.y, pt.z);
      }
    }

    // For now, assume uniform timestamps
    timestamps.clear();
    timestamps.resize(points.size());
    for (size_t i = 0; i < timestamps.size(); ++i) {
      timestamps[i] = static_cast<float>(i) / static_cast<float>(timestamps.size());
    }
  }

  void publishOdometry(const OdometryResult& result, const std_msgs::msg::Header& header) {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Extract position and orientation from pose matrix
    Eigen::Vector3f position = result.pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = result.pose.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotation);

    odom_msg.pose.pose.position.x = position.x();
    odom_msg.pose.pose.position.y = position.y();
    odom_msg.pose.pose.position.z = position.z();

    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();

    // Set covariance (simplified)
    for (int i = 0; i < 36; i++) {
      odom_msg.pose.covariance[i] = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[14] = 0.01;  // z
    odom_msg.pose.covariance[21] = 0.01;  // roll
    odom_msg.pose.covariance[28] = 0.01;  // pitch
    odom_msg.pose.covariance[35] = 0.01;  // yaw

    odom_pub_->publish(odom_msg);
  }

  void publishLocalMap(const PointCloud& map_points, const std_msgs::msg::Header& header) {
    if (map_points.empty()) return;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.points.reserve(map_points.size());

    for (const auto& pt : map_points) {
      pcl_cloud.points.emplace_back(pt.x, pt.y, pt.z);
    }

    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.stamp = header.stamp;
    cloud_msg.header.frame_id = odom_frame_;

    map_pub_->publish(cloud_msg);
  }

  void publishCurrentScan(const PointCloud& scan_points, const std_msgs::msg::Header& header) {
    if (scan_points.empty()) return;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.points.reserve(scan_points.size());

    for (const auto& pt : scan_points) {
      pcl_cloud.points.emplace_back(pt.x, pt.y, pt.z);
    }

    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.stamp = header.stamp;
    cloud_msg.header.frame_id = base_frame_;

    scan_pub_->publish(cloud_msg);
  }

  void publishPose(const OdometryResult& result, const std_msgs::msg::Header& header) {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = header.stamp;
    pose_msg.header.frame_id = odom_frame_;

    Eigen::Vector3f position = result.pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = result.pose.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotation);

    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();

    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();

    pose_pub_->publish(pose_msg);
  }

  void publishTF(const OdometryResult& result, const std_msgs::msg::Header& header) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = header.stamp;
    transform_stamped.header.frame_id = odom_frame_;
    transform_stamped.child_frame_id = base_frame_;

    Eigen::Vector3f position = result.pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = result.pose.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotation);

    transform_stamped.transform.translation.x = position.x();
    transform_stamped.transform.translation.y = position.y();
    transform_stamped.transform.translation.z = position.z();

    transform_stamped.transform.rotation.x = quaternion.x();
    transform_stamped.transform.rotation.y = quaternion.y();
    transform_stamped.transform.rotation.z = quaternion.z();
    transform_stamped.transform.rotation.w = quaternion.w();

    tf_broadcaster_->sendTransform(transform_stamped);
  }
};
