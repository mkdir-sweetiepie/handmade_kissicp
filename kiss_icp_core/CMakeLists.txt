cmake_minimum_required(VERSION 3.16)
project(kiss_icp_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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

# 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/kiss_icp.cpp
  src/local_map.cpp
  ../adaptive_threshold/src/adaptive_threshold.cpp
  ../constant_velocity/src/constant_velocity_model.cpp
  ../kiss_icp_voxel/src/voxel_grid.cpp
  ../robust_icp/src/robust_icp.cpp
  ../scan_deskewing/src/scan_deskewer.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

# 라이브러리 설치
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 헤더 파일 설치 - 이 부분이 중요!
install(DIRECTORY include/
  DESTINATION include/
)

# 의존 모듈들의 헤더도 함께 설치
install(DIRECTORY
  ../adaptive_threshold/include/
  ../constant_velocity/include/
  ../kiss_icp_voxel/include/
  ../robust_icp/include/
  ../scan_deskewing/include/
  DESTINATION include/
)

# Export 설정
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 PCL)
ament_export_include_directories(include)

ament_package()
