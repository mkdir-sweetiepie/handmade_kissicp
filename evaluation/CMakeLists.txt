cmake_minimum_required(VERSION 3.16)
project(evaluation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# 헤더 포함 디렉토리 설정
include_directories(
  include
  ../kiss_icp_core/include
  ../adaptive_threshold/include
  ../constant_velocity/include
  ../kiss_icp_voxel/include
  ../robust_icp/include
  ../scan_deskewing/include
  ${PCL_INCLUDE_DIRS}
)

# 링크 디렉토리 설정
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/odometry_evaluator.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

# 완전한 KITTI 평가 실행파일
add_executable(complete_kitti_evaluation
  src/complete_kitti_evaluation.cpp
)

target_link_libraries(complete_kitti_evaluation
  ${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

# 기본 테스트 실행파일
add_executable(test_evaluation src/test_evaluation.cpp)
target_link_libraries(test_evaluation ${PROJECT_NAME})

# 간단한 테스트 실행파일
add_executable(simple_test
  src/simple_test.cpp
)

target_link_libraries(simple_test ${PROJECT_NAME})

# 설치
install(TARGETS ${PROJECT_NAME} complete_kitti_evaluation test_evaluation simple_test
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 PCL)

ament_package()
