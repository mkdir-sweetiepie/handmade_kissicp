cmake_minimum_required(VERSION 3.10)
project(adaptive_threshold)

# C++14 사용
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen3 찾기
find_package(Eigen3 REQUIRED)

# PCL 라이브러리 찾기 (선택적)
find_package(PCL COMPONENTS common QUIET)

# 헤더 포함 디렉토리 설정
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# adaptive_threshold 라이브러리 생성
add_library(adaptive_threshold src/adaptive_threshold.cpp)
target_link_libraries(adaptive_threshold ${EIGEN3_LIBRARIES})

# 테스트 실행 파일 생성
add_executable(test_adaptive_threshold src/test_adaptive_threshold.cpp)
target_link_libraries(test_adaptive_threshold adaptive_threshold ${EIGEN3_LIBRARIES})

# 설치 설정 (선택적)
install(TARGETS adaptive_threshold
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)
