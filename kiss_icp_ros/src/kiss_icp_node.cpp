#include "kiss_icp_ros/kiss_icp_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kiss_icp_ros {

KissICPNode::KissICPNode() : Node("kiss_icp_node") {
  // KISS-ICP 핵심 모듈 초기화
  kiss_icp_ = std::make_unique<kiss_icp_core::KissICP>();

  // 파라미터 설정
  float max_range = this->declare_parameter("max_range", 100.0);
  kiss_icp_->setMaxRange(max_range);

  // ROS 2 인터페이스 초기화
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 10, std::bind(&KissICPNode::cloudCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 1);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  path_msg_.header.frame_id = "odom";

  RCLCPP_INFO(this->get_logger(), "KISS-ICP Node 초기화 완료 - 5가지 핵심 모듈 활성화");
}

void KissICPNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // ROS 포인트클라우드를 KISS-ICP 형식으로 변환
  auto kiss_cloud = rosToKissICP(*msg);

  // 타임스탬프 생성 (간단한 버전)
  std::vector<float> timestamps(kiss_cloud.size(), 0.0f);

  // KISS-ICP 처리 (5가지 모듈 모두 사용)
  auto result = kiss_icp_->processFrame(kiss_cloud, timestamps);

  if (result.success) {
    publishResults(result, msg->header);
  }
}

kiss_icp_core::PointCloud KissICPNode::rosToKissICP(const sensor_msgs::msg::PointCloud2& ros_cloud) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(ros_cloud, pcl_cloud);

  kiss_icp_core::PointCloud kiss_cloud;
  kiss_cloud.reserve(pcl_cloud.points.size());

  for (const auto& pcl_point : pcl_cloud.points) {
    kiss_cloud.emplace_back(pcl_point.x, pcl_point.y, pcl_point.z);
  }

  return kiss_cloud;
}

sensor_msgs::msg::PointCloud2 KissICPNode::kissICPToRos(const kiss_icp_core::PointCloud& kiss_cloud, const std::string& frame_id) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.points.reserve(kiss_cloud.size());

  for (const auto& point : kiss_cloud) {
    pcl_cloud.points.emplace_back(point.x, point.y, point.z);
  }

  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ros_cloud.header.frame_id = frame_id;

  return ros_cloud;
}

void KissICPNode::publishResults(const kiss_icp_core::OdometryResult& result, const std_msgs::msg::Header& header) {
  // 1. 오도메트리 발행
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = header;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // 포즈 변환
  auto pose = result.pose;
  odom_msg.pose.pose.position.x = pose(0, 3);
  odom_msg.pose.pose.position.y = pose(1, 3);
  odom_msg.pose.pose.position.z = pose(2, 3);

  Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
  Eigen::Quaternionf quat(rotation);
  odom_msg.pose.pose.orientation.x = quat.x();
  odom_msg.pose.pose.orientation.y = quat.y();
  odom_msg.pose.pose.orientation.z = quat.z();
  odom_msg.pose.pose.orientation.w = quat.w();

  odom_pub_->publish(odom_msg);

  // 2. 경로 발행
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;

  path_msg_.header.stamp = header.stamp;
  path_msg_.poses.push_back(pose_stamped);
  path_pub_->publish(path_msg_);

  // 3. **LOCAL MAP 발행 임시 비활성화**
  // TODO: OdometryResult 구조체에 적절한 멤버가 확인되면 구현
  RCLCPP_WARN_ONCE(this->get_logger(), "Local Map 발행 기능 임시 비활성화됨");

  // 4. TF 발행
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom_msg.header;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = pose(0, 3);
  transform.transform.translation.y = pose(1, 3);
  transform.transform.translation.z = pose(2, 3);
  transform.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(transform);

  RCLCPP_INFO(this->get_logger(), "4가지 모듈 처리 완료 - 위치: [%.2f, %.2f, %.2f]", pose(0, 3), pose(1, 3), pose(2, 3));
}

}  // namespace kiss_icp_ros