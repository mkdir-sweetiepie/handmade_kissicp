cmake_minimum_required(VERSION 3.10)
project(robust_icp)

# C++14 사용
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL 라이브러리 찾기
find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree visualization)

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

# robust_icp 라이브러리 생성
add_library(robust_icp src/robust_icp.cpp)
target_link_libraries(robust_icp ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

# 테스트 실행 파일 생성
add_executable(test_robust_icp src/test_robust_icp.cpp)
target_link_libraries(test_robust_icp robust_icp ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

# 설치 설정 (선택적)
install(TARGETS robust_icp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)
