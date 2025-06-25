cmake_minimum_required(VERSION 3.16)
project(kiss_icp_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

# kiss_icp_core 찾기
find_package(kiss_icp_core REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

# 실행파일 생성 - 모든 소스 파일 포함
add_executable(kiss_icp_ros_node
  src/main.cpp
  src/kiss_icp_node.cpp
  src/kitti_player_node.cpp
)

# FIXED: PCL만 직접 링크하고, ROS 2 패키지는 ament_target_dependencies 사용
target_link_libraries(kiss_icp_ros_node
  ${PCL_LIBRARIES}
)

# ROS 2 의존성 + kiss_icp_core를 여기서 처리
ament_target_dependencies(kiss_icp_ros_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  kiss_icp_core
)

# 실행파일 설치
install(TARGETS kiss_icp_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

# 디렉토리가 존재할 때만 설치
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

ament_package()
