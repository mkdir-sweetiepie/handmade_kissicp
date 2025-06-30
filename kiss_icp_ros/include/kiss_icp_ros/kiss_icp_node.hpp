#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

#include "kiss_icp_core/kiss_icp.h"

namespace kiss_icp_ros {

class KissICPNode : public rclcpp::Node {
public:
    KissICPNode();

private:
    std::unique_ptr<kiss_icp_core::KissICP> kiss_icp_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Path path_msg_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    kiss_icp_core::PointCloud rosToKissICP(const sensor_msgs::msg::PointCloud2& ros_cloud);
    sensor_msgs::msg::PointCloud2 kissICPToRos(const kiss_icp_core::PointCloud& kiss_cloud, const std::string& frame_id);
    void publishResults(const kiss_icp_core::OdometryResult& result, const std_msgs::msg::Header& header);
};

} // namespace kiss_icp_ros
