// kiss_icp_ros/src/kitti_player_node.cpp
#include "kiss_icp_ros/kitti_player_node.hpp"

namespace kiss_icp_ros {

KittiPlayerNode::KittiPlayerNode(const rclcpp::NodeOptions& options)
    : Node("kitti_player_node", options), current_scan_index_(0), is_playing_(false) {
  
  RCLCPP_INFO(get_logger(), "Starting KITTI Player Node");

  declareParameters();
  
  if (!loadSequence()) {
    RCLCPP_ERROR(get_logger(), "Failed to load KITTI sequence");
    return;
  }

  // Initialize publisher
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

  // Initialize timer
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = create_wall_timer(period, std::bind(&KittiPlayerNode::publishCallback, this));

  if (auto_start_) {
    startPlayback();
  }

  RCLCPP_INFO(get_logger(), "KITTI Player initialized successfully");
  RCLCPP_INFO(get_logger(), "  Loaded %zu frames from sequence %s", scan_files_.size(), sequence_.c_str());
  RCLCPP_INFO(get_logger(), "  Publishing at %.1f Hz", publish_rate_);
}

KittiPlayerNode::~KittiPlayerNode() {
  RCLCPP_INFO(get_logger(), "Shutting down KITTI Player Node");
}

void KittiPlayerNode::play() {
  startPlayback();
}

void KittiPlayerNode::stop() {
  stopPlayback();
}

void KittiPlayerNode::reset() {
  resetPlayback();
}

void KittiPlayerNode::declareParameters() {
  // Dataset parameters
  declare_parameter("dataset_path", "");
  declare_parameter("sequence", "00");
  declare_parameter("publish_rate", 10.0);
  declare_parameter("frame_id", "velodyne");
  declare_parameter("loop_playback", true);
  declare_parameter("auto_start", false);

  // Get parameters
  dataset_path_ = get_parameter("dataset_path").as_string();
  sequence_ = get_parameter("sequence").as_string();
  publish_rate_ = get_parameter("publish_rate").as_double();
  frame_id_ = get_parameter("frame_id").as_string();
  loop_playback_ = get_parameter("loop_playback").as_bool();
  auto_start_ = get_parameter("auto_start").as_bool();

  if (dataset_path_.empty()) {
    RCLCPP_ERROR(get_logger(), "dataset_path parameter is required");
    return;
  }

  RCLCPP_INFO(get_logger(), "Parameters loaded:");
  RCLCPP_INFO(get_logger(), "  dataset_path: %s", dataset_path_.c_str());
  RCLCPP_INFO(get_logger(), "  sequence: %s", sequence_.c_str());
  RCLCPP_INFO(get_logger(), "  publish_rate: %.1f Hz", publish_rate_);
  RCLCPP_INFO(get_logger(), "  frame_id: %s", frame_id_.c_str());
}

bool KittiPlayerNode::loadSequence() {
  namespace fs = std::filesystem;

  // Construct path to velodyne folder
  std::string velodyne_path;
  if (sequence_.empty()) {
    velodyne_path = dataset_path_ + "/velodyne";
  } else {
    velodyne_path = dataset_path_ + "/" + sequence_ + "/velodyne";
  }

  if (!fs::exists(velodyne_path)) {
    RCLCPP_ERROR(get_logger(), "Velodyne folder not found: %s", velodyne_path.c_str());
    return false;
  }

  // Collect all .bin files
  scan_files_.clear();
  try {
    for (const auto& entry : fs::directory_iterator(velodyne_path)) {
      if (entry.path().extension() == ".bin") {
        scan_files_.push_back(entry.path().string());
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error reading directory %s: %s", velodyne_path.c_str(), e.what());
    return false;
  }

  if (scan_files_.empty()) {
    RCLCPP_ERROR(get_logger(), "No .bin files found in %s", velodyne_path.c_str());
    return false;
  }

  // Sort files numerically
  std::sort(scan_files_.begin(), scan_files_.end());

  RCLCPP_INFO(get_logger(), "Successfully loaded %zu scan files", scan_files_.size());
  return true;
}

void KittiPlayerNode::publishCallback() {
  if (!is_playing_ || scan_files_.empty()) {
    return;
  }

  if (current_scan_index_ >= scan_files_.size()) {
    if (loop_playback_) {
      current_scan_index_ = 0;
      RCLCPP_INFO(get_logger(), "Looping back to first frame");
    } else {
      RCLCPP_INFO(get_logger(), "Reached end of sequence, stopping playback");
      stopPlayback();
      return;
    }
  }

  // Publish current scan
  auto timestamp = now();
  publishScan(scan_files_[current_scan_index_], timestamp);

  RCLCPP_DEBUG(get_logger(), "Published frame %zu/%zu", current_scan_index_ + 1, scan_files_.size());
  current_scan_index_++;
}

void KittiPlayerNode::publishScan(const std::string& scan_file, const rclcpp::Time& timestamp) {
  // Load point cloud
  auto cloud = loadKittiBinaryFile(scan_file);
  if (!cloud || cloud->empty()) {
    RCLCPP_WARN(get_logger(), "Failed to load or empty scan: %s", scan_file.c_str());
    return;
  }

  // Convert to ROS message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = timestamp;
  cloud_msg.header.frame_id = frame_id_;

  // Publish
  cloud_pub_->publish(cloud_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KittiPlayerNode::loadKittiBinaryFile(const std::string& filepath) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::ifstream file(filepath, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Cannot open file: %s", filepath.c_str());
    return nullptr;
  }

  // KITTI binary format: each point is 4 floats (x, y, z, intensity)
  float point_data[4];
  while (file.read(reinterpret_cast<char*>(point_data), sizeof(point_data))) {
    // Check for valid coordinates
    if (std::isfinite(point_data[0]) && std::isfinite(point_data[1]) && std::isfinite(point_data[2])) {
      cloud->points.emplace_back(point_data[0], point_data[1], point_data[2]);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  if (cloud->empty()) {
    RCLCPP_WARN(get_logger(), "Loaded empty point cloud from: %s", filepath.c_str());
  }

  return cloud;
}

void KittiPlayerNode::startPlayback() {
  if (scan_files_.empty()) {
    RCLCPP_WARN(get_logger(), "No scan files loaded, cannot start playback");
    return;
  }

  is_playing_ = true;
  start_time_ = now();
  RCLCPP_INFO(get_logger(), "Started playback of %zu frames", scan_files_.size());
}

void KittiPlayerNode::stopPlayback() {
  is_playing_ = false;
  RCLCPP_INFO(get_logger(), "Stopped playback");
}

void KittiPlayerNode::resetPlayback() {
  current_scan_index_ = 0;
  start_time_ = now();
  RCLCPP_INFO(get_logger(), "Reset playback to beginning");
}

} // namespace kiss_icp_ros