#pragma once

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace kiss_icp_ros {

class KittiPlayerNode : public rclcpp::Node {
public:
  explicit KittiPlayerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~KittiPlayerNode();

  void play();
  void stop();
  void reset();
  bool isPlaying() const { return is_playing_; }
  size_t getCurrentScanIndex() const { return current_scan_index_; }
  size_t getTotalScans() const { return scan_files_.size(); }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string dataset_path_;
  std::string sequence_;
  double publish_rate_;
  bool loop_playback_;
  bool auto_start_;
  std::string frame_id_;

  std::vector<std::string> scan_files_;
  size_t current_scan_index_;
  bool is_playing_;
  rclcpp::Time start_time_;

  void declareParameters();
  bool loadSequence();
  void publishCallback();
  void publishScan(const std::string& scan_file, const rclcpp::Time& timestamp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr loadKittiBinaryFile(const std::string& filepath);
  void startPlayback();
  void stopPlayback();
  void resetPlayback();
};

} // namespace kiss_icp_ros