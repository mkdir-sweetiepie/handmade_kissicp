cmake_minimum_required(VERSION 3.10)
project(kiss_icp_voxel)

# C++14 사용
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL 라이브러리 찾기
find_package(PCL REQUIRED COMPONENTS common io)

# 헤더 포함 디렉토리 설정
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# PCL 라이브러리 링크 설정
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# voxel_grid 라이브러리 생성
add_library(voxel_grid src/voxel_grid.cpp)
target_link_libraries(voxel_grid ${PCL_LIBRARIES})

# 테스트 실행 파일 생성
add_executable(test_voxel_grid src/test_voxel_grid.cpp)
target_link_libraries(test_voxel_grid voxel_grid ${PCL_LIBRARIES})

install(TARGETS voxel_grid
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)