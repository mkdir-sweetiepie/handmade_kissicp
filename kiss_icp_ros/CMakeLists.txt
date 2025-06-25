cmake_minimum_required(VERSION 3.16)
project(kiss_icp_ros)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(Eigen3 REQUIRED)

# Find our custom package
find_package(kiss_icp_core REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Create executable
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/kiss_icp_node.cpp
  src/kitti_player_node.cpp
)

# Set target properties
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Add ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  kiss_icp_core
)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (only if they exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.py" PATTERN "*.yaml"
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml" PATTERN "*.json"
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
    FILES_MATCHING PATTERN "*.rviz"
  )
endif()

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export information
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  kiss_icp_core
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
