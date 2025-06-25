#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class KITTIPlayerNode : public rclcpp::Node {
 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> scan_files_;
  size_t current_frame_;
  double publish_rate_;
  std::string frame_id_;

 public:
  KITTIPlayerNode() : Node("kitti_player_node"), current_frame_(0) {
    declare_parameter("data_path", "");
    declare_parameter("publish_rate", 10.0);
    declare_parameter("frame_id", "velodyne");
    declare_parameter("loop", true);

    std::string data_path = get_parameter("data_path").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    frame_id_ = get_parameter("frame_id").as_string();

    if (data_path.empty()) {
      RCLCPP_ERROR(get_logger(), "data_path parameter is required");
      return;
    }

    if (!loadKITTISequence(data_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to load KITTI sequence from: %s", data_path.c_str());
      return;
    }

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(period, std::bind(&KITTIPlayerNode::publishNextFrame, this));

    RCLCPP_INFO(get_logger(), "KITTI Player initialized with %zu frames", scan_files_.size());
  }

 private:
  bool loadKITTISequence(const std::string& sequence_path) {
    namespace fs = std::filesystem;

    std::string velodyne_path = sequence_path + "/velodyne";
    if (!fs::exists(velodyne_path)) {
      RCLCPP_ERROR(get_logger(), "Velodyne folder not found: %s", velodyne_path.c_str());
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

  void publishNextFrame() {
    if (scan_files_.empty()) return;

    if (current_frame_ >= scan_files_.size()) {
      bool loop = get_parameter("loop").as_bool();
      if (loop) {
        current_frame_ = 0;
        RCLCPP_INFO(get_logger(), "Looping back to first frame");
      } else {
        RCLCPP_INFO(get_logger(), "Reached end of sequence");
        timer_->cancel();
        return;
      }
    }

    auto cloud = loadKITTIPointCloud(scan_files_[current_frame_]);
    if (!cloud) {
      RCLCPP_WARN(get_logger(), "Failed to load frame %zu", current_frame_);
      current_frame_++;
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = frame_id_;

    cloud_pub_->publish(cloud_msg);
    current_frame_++;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr loadKITTIPointCloud(const std::string& filename) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    std::ifstream file(filename, std::ios::binary);
    if (!file) {
      return nullptr;
    }

    float point[4];
    while (file.read(reinterpret_cast<char*>(point), sizeof(point))) {
      if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
        cloud->points.emplace_back(point[0], point[1], point[2]);
      }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
  }
};
