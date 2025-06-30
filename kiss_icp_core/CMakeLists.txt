cmake_minimum_required(VERSION 3.16)
project(kiss_icp_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# 모든 헤더 포함
include_directories(
  include
  ../adaptive_threshold/include
  ../constant_velocity/include
  ../kiss_icp_voxel/include
  ../robust_icp/include
  ../scan_deskewing/include
  ${PCL_INCLUDE_DIRS}
)

# 모든 모듈을 포함한 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/kiss_icp.cpp
  src/local_map.cpp
  ../adaptive_threshold/src/adaptive_threshold.cpp
  ../constant_velocity/src/constant_velocity_model.cpp
  ../kiss_icp_voxel/src/voxel_grid.cpp
  ../robust_icp/src/robust_icp.cpp
  ../scan_deskewing/src/scan_deskewer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../adaptive_threshold/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../constant_velocity/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../kiss_icp_voxel/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../robust_icp/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../scan_deskewing/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 PCL)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  ../adaptive_threshold/include/
  ../constant_velocity/include/
  ../kiss_icp_voxel/include/
  ../robust_icp/include/
  ../scan_deskewing/include/
  DESTINATION include/
)

ament_package()
