#include "kiss_icp_ros/kitti_player_node.hpp"

namespace kiss_icp_ros {

KittiPlayerNode::KittiPlayerNode(const rclcpp::NodeOptions& options) : Node("kitti_player_node", options), current_scan_index_(0), is_playing_(false) {
  declareParameters();

  if (dataset_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "dataset_path parameter is required");
    return;
  }

  if (!loadSequence()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load KITTI sequence from: %s", dataset_path_.c_str());
    return;
  }

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(period, std::bind(&KittiPlayerNode::publishCallback, this));

  if (auto_start_) {
    startPlayback();
  }

  RCLCPP_INFO(this->get_logger(), "KITTI Player initialized with %zu frames", scan_files_.size());
}

void KittiPlayerNode::declareParameters() {
  dataset_path_ = this->declare_parameter("dataset_path", std::string(""));
  publish_rate_ = this->declare_parameter("publish_rate", 10.0);
  frame_id_ = this->declare_parameter("frame_id", std::string("velodyne"));
  loop_playback_ = this->declare_parameter("loop_playback", true);
  auto_start_ = this->declare_parameter("auto_start", true);
}

bool KittiPlayerNode::loadSequence() {
  namespace fs = std::filesystem;

  std::string velodyne_path = dataset_path_ + "/velodyne";
  if (!fs::exists(velodyne_path)) {
    RCLCPP_ERROR(this->get_logger(), "Velodyne folder not found: %s", velodyne_path.c_str());
    return false;
  }

  scan_files_.clear();
  for (const auto& entry : fs::directory_iterator(velodyne_path)) {
    if (entry.path().extension() == ".bin") {
      scan_files_.push_back(entry.path().string());
    }
  }

  std::sort(scan_files_.begin(), scan_files_.end());
  return !scan_files_.empty();
}

void KittiPlayerNode::publishCallback() {
  if (!is_playing_ || scan_files_.empty()) return;

  if (current_scan_index_ >= scan_files_.size()) {
    if (loop_playback_) {
      current_scan_index_ = 0;
      RCLCPP_INFO(this->get_logger(), "Looping back to first frame");
    } else {
      RCLCPP_INFO(this->get_logger(), "Reached end of sequence");
      stopPlayback();
      return;
    }
  }

  auto cloud = loadKittiBinaryFile(scan_files_[current_scan_index_]);
  if (!cloud || cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to load frame %zu", current_scan_index_);
    current_scan_index_++;
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = this->get_clock()->now();
  cloud_msg.header.frame_id = frame_id_;

  cloud_pub_->publish(cloud_msg);
  current_scan_index_++;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KittiPlayerNode::loadKittiBinaryFile(const std::string& filepath) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::ifstream file(filepath, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", filepath.c_str());
    return nullptr;
  }

  float point[4];  // x, y, z, intensity
  while (file.read(reinterpret_cast<char*>(point), sizeof(point))) {
    if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
      cloud->points.emplace_back(point[0], point[1], point[2]);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;

  return cloud;
}

void KittiPlayerNode::play() { startPlayback(); }
void KittiPlayerNode::stop() { stopPlayback(); }
void KittiPlayerNode::reset() { resetPlayback(); }

void KittiPlayerNode::startPlayback() {
  is_playing_ = true;
  RCLCPP_INFO(this->get_logger(), "Playback started");
}

void KittiPlayerNode::stopPlayback() {
  is_playing_ = false;
  RCLCPP_INFO(this->get_logger(), "Playback stopped");
}

void KittiPlayerNode::resetPlayback() {
  current_scan_index_ = 0;
  RCLCPP_INFO(this->get_logger(), "Playback reset to beginning");
}

}  // namespace kiss_icp_ros