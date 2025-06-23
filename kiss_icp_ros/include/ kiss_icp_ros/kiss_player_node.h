#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

namespace kiss_icp_ros {

/**
 * @brief ROS 2 node for playing back KITTI dataset sequences
 *
 * This node reads KITTI dataset point cloud files and publishes them
 * as ROS 2 sensor_msgs/PointCloud2 messages with appropriate timing.
 */
class KittiPlayerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   */
  KittiPlayerNode();

  /**
   * @brief Destructor
   */
  ~KittiPlayerNode();

 private:
  // ROS 2 publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  // Timer for publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Dataset parameters
  std::string dataset_path_;
  std::string sequence_;
  double publish_rate_;
  bool loop_playback_;
  bool auto_start_;

  // Playback state
  std::vector<std::string> scan_files_;
  size_t current_scan_index_;
  bool is_playing_;
  rclcpp::Time start_time_;

  // Frame information
  std::string frame_id_;

  /**
   * @brief Declare and load ROS parameters
   */
  void declareParameters();

  /**
   * @brief Load KITTI sequence information
   * @return True if successful
   */
  bool loadSequence();

  /**
   * @brief Timer callback to publish next scan
   */
  void publishCallback();

  /**
   * @brief Load and publish a single scan
   * @param scan_file Path to the scan file
   * @param timestamp Timestamp for the message
   */
  void publishScan(const std::string& scan_file, const rclcpp::Time& timestamp);

  /**
   * @brief Load point cloud from KITTI binary file
   * @param filepath Path to the binary file
   * @return Loaded point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr loadKittiBinaryFile(const std::string& filepath);

  /**
   * @brief Start playback
   */
  void startPlayback();

  /**
   * @brief Stop playback
   */
  void stopPlayback();

  /**
   * @brief Reset playback to beginning
   */
  void resetPlayback();

 public:
  /**
   * @brief Control playback (for external control)
   */
  void play() { startPlayback(); }
  void stop() { stopPlayback(); }
  void reset() { resetPlayback(); }

  /**
   * @brief Get playback status
   */
  bool isPlaying() const { return is_playing_; }
  size_t getCurrentScanIndex() const { return current_scan_index_; }
  size_t getTotalScans() const { return scan_files_.size(); }
  double getProgress() const { return scan_files_.empty() ? 0.0 : static_cast<double>(current_scan_index_) / scan_files_.size(); }
};

}  // namespace kiss_icp_ros