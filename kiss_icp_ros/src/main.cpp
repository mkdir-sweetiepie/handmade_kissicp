// kiss_icp_ros/src/main.cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "kiss_icp_ros/kiss_icp_node.hpp"
#include "kiss_icp_ros/kitti_player_node.hpp"

void printUsage(const std::string& program_name) {
  std::cout << "Usage: " << program_name << " [mode]\n";
  std::cout << "Modes:\n";
  std::cout << "  (default)  - Run KISS-ICP odometry node\n";
  std::cout << "  player     - Run KITTI dataset player\n";
  std::cout << "  --help     - Show this help message\n";
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    // Parse command line arguments
    std::string mode = "odometry";
    if (argc > 1) {
      std::string arg = argv[1];
      if (arg == "--help" || arg == "-h") {
        printUsage(argv[0]);
        return 0;
      } else if (arg == "player") {
        mode = "player";
      } else if (arg == "odometry") {
        mode = "odometry";
      } else {
        std::cerr << "Unknown mode: " << arg << std::endl;
        printUsage(argv[0]);
        return 1;
      }
    }

    // Create and run the appropriate node
    std::shared_ptr<rclcpp::Node> node;

    if (mode == "player") {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Starting KITTI Player Node");
      node = std::make_shared<kiss_icp_ros::KittiPlayerNode>();
    } else {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Starting KISS-ICP Odometry Node");
      node = std::make_shared<kiss_icp_ros::KissICPNode>();
    }

    // Set up signal handlers for graceful shutdown
    rclcpp::install_signal_handlers();

    // Spin the node
    RCLCPP_INFO(rclcpp::get_logger("main"), "Node started successfully, spinning...");
    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Node shutdown complete");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown exception caught");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}