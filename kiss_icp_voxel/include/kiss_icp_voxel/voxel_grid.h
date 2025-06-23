#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>

// 3차원 정수 인덱스를 저장하는 구조체
struct VoxelKey {
  int x;
  int y;
  int z;
};

// 두 VoxelKey를 비교하는 함수 (map에서 정렬 기준으로 사용)
bool compareVoxelKeys(const VoxelKey& a, const VoxelKey& b);

class VoxelGrid {
 public:
  // 생성자
  VoxelGrid(float size);

  // 다운샘플링 함수
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

 private:
  float voxel_size;  // 복셀 크기

  // 포인트로부터 복셀 키 계산
  VoxelKey getVoxelKey(const pcl::PointXYZ& point) const;
};