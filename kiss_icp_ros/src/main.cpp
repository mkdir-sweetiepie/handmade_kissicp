// kiss_icp_ros/src/main.cpp
#include <rclcpp/rclcpp.hpp>

#include "kiss_icp_node.cpp"
#include "kitti_player_node.cpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc > 1 && std::string(argv[1]) == "player") {
    auto node = std::make_shared<KITTIPlayerNode>();
    rclcpp::spin(node);
  } else {
    auto node = std::make_shared<KissICPNode>();
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}