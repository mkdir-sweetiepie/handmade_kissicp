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
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", rclcpp::SensorDataQoS(),
        std::bind(&KissICPNode::pointCloudCallback, this, std::placeholders::_1));

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "KISS-ICP Node initialized");
  }

 private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      // Convert ROS message to internal format
      auto [points, timestamps] = convertPointCloud(msg);
      
      if (points.empty()) {
        RCLCPP_WARN(get_logger(), "Empty point cloud received");
        return;
      }

      // Process with KISS-ICP
      auto result = odometry_.processFrame(points, timestamps);

      if (!result.success) {
        RCLCPP_WARN(get_logger(), "KISS-ICP processing failed");
        return;
      }

      // Publish results
      publishOdometry(result, msg->header);
      publishPose(result, msg->header);
      publishTransforms(result, msg->header);
      
      if (!result.local_map_points.empty()) {
        publishLocalMap(result, msg->header);
      }
      
      publishCurrentScan(points, msg->header);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error processing point cloud: %s", e.what());
    }
  }

  std::pair<PointCloud, std::vector<float>> convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloud points;
    std::vector<float> timestamps;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    points.reserve(pcl_cloud.size());
    timestamps.reserve(pcl_cloud.size());

    for (const auto& pcl_point : pcl_cloud) {
      if (std::isfinite(pcl_point.x) && std::isfinite(pcl_point.y) && std::isfinite(pcl_point.z)) {
        Point3D point;
        point.x = pcl_point.x;
        point.y = pcl_point.y;
        point.z = pcl_point.z;
        points.push_back(point);
        timestamps.push_back(0.0f);
      }
    }

    return {points, timestamps};
  }

  void publishOdometry(const OdometryResult& result, const std_msgs::msg::Header& header) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = header;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Set pose from transformation matrix
    auto pose = eigenToPose(result.pose);
    odom_msg.pose.pose = pose;

    odom_pub_->publish(odom_msg);
  }

  void publishPose(const OdometryResult& result, const std_msgs::msg::Header& header) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = eigenToPose(result.pose);

    pose_pub_->publish(pose_msg);
  }

  void publishTransforms(const OdometryResult& result, const std_msgs::msg::Header& header) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header = header;
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = base_frame_;
    transform_msg.transform = eigenToTransform(result.pose);

    tf_broadcaster_->sendTransform(transform_msg);
  }

  void publishLocalMap(const OdometryResult& result, const std_msgs::msg::Header& header) {
    auto map_msg = convertToROS(result.local_map_points, header);
    map_msg.header.frame_id = odom_frame_;
    map_pub_->publish(map_msg);
  }

  void publishCurrentScan(const PointCloud& points, const std_msgs::msg::Header& header) {
    auto scan_msg = convertToROS(points, header);
    scan_pub_->publish(scan_msg);
  }

  sensor_msgs::msg::PointCloud2 convertToROS(const PointCloud& points, const std_msgs::msg::Header& header) {
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

  geometry_msgs::msg::Pose eigenToPose(const Eigen::Matrix4f& matrix) {
    geometry_msgs::msg::Pose pose;
    
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    
    Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    
    return pose;
  }

  geometry_msgs::msg::Transform eigenToTransform(const Eigen::Matrix4f& matrix) {
    geometry_msgs::msg::Transform transform;
    
    transform.translation.x = matrix(0, 3);
    transform.translation.y = matrix(1, 3);
    transform.translation.z = matrix(2, 3);
    
    Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    
    transform.rotation.x = quat.x();
    transform.rotation.y = quat.y();
    transform.rotation.z = quat.z();
    transform.rotation.w = quat.w();
    
    return transform;
  }
};
