#include <rclcpp/rclcpp.hpp>

// 임시로 cpp 파일 직접 include
#include "kiss_icp_node.cpp"
#include "kitti_player_node.cpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    if (argc > 1 && std::string(argv[1]) == "player") {
      auto node = std::make_shared<KITTIPlayerNode>();
      RCLCPP_INFO(rclcpp::get_logger("main"), "Starting KITTI Player Node");
      rclcpp::spin(node);
    } else {
      auto node = std::make_shared<KissICPNode>();
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
