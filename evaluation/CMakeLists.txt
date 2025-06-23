cmake_minimum_required(VERSION 3.16)
project(evaluation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/odometry_evaluator.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# 테스트 실행파일
add_executable(test_evaluation src/test_evaluation.cpp)
target_link_libraries(test_evaluation ${PROJECT_NAME})

# 설치
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS test_evaluation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

ament_package()
