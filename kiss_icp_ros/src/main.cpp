#include <rclcpp/rclcpp.hpp>

#include "kiss_icp_ros/kiss_icp_node.hpp"
#include "kiss_icp_ros/kitti_player_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    if (argc > 1 && std::string(argv[1]) == "player") {
      auto node = std::make_shared<kiss_icp_ros::KittiPlayerNode>();
      RCLCPP_INFO(rclcpp::get_logger("main"), "Starting KITTI Player Node");
      rclcpp::spin(node);
    } else {
      auto node = std::make_shared<kiss_icp_ros::KissICPNode>();
      RCLCPP_INFO(rclcpp::get_logger("main"), "Starting KISS-ICP Node");
      rclcpp::spin(node);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Node failed to start: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}