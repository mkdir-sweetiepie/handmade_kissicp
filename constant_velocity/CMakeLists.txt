cmake_minimum_required(VERSION 3.10)
project(constant_velocity)

# C++14 사용
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen 라이브러리 찾기
find_package(Eigen3 REQUIRED)

# 헤더 포함 디렉토리 설정
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# constant_velocity_model 라이브러리 생성
add_library(constant_velocity_model src/constant_velocity_model.cpp)
target_link_libraries(constant_velocity_model ${EIGEN3_LIBRARIES})

# 테스트 실행 파일 생성
add_executable(test_velocity_model src/test_velocity_model.cpp)
target_link_libraries(test_velocity_model constant_velocity_model)

install(TARGETS constant_velocity_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)