cmake_minimum_required(VERSION 3.10)
project(scan_deskewing)

# C++14 사용
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL 라이브러리 찾기
find_package(PCL REQUIRED COMPONENTS common io)

# Eigen 라이브러리 찾기
find_package(Eigen3 REQUIRED)

# 헤더 포함 디렉토리 설정
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 라이브러리 링크 디렉토리
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# scan_deskewer 라이브러리 생성
add_library(scan_deskewer src/scan_deskewer.cpp)
target_link_libraries(scan_deskewer ${PCL_LIBRARIES})

# 테스트 실행 파일 생성
add_executable(test_scan_deskewing src/test_scan_deskewing.cpp)
target_link_libraries(test_scan_deskewing scan_deskewer ${PCL_LIBRARIES})

install(TARGETS scan_deskewer
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)