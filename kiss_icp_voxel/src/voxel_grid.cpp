#include "../include/kiss_icp_voxel/voxel_grid.h"

#include <cmath>

// 두 VoxelKey를 비교하는 함수 구현
bool compareVoxelKeys(const VoxelKey& a, const VoxelKey& b) {
  if (a.x != b.x) return a.x < b.x;
  if (a.y != b.y) return a.y < b.y;
  return a.z < b.z;
}

// 생성자 구현
VoxelGrid::VoxelGrid(float size) : voxel_size(size) {}

// 포인트로부터 복셀 키 계산 함수 구현
VoxelKey VoxelGrid::getVoxelKey(const pcl::PointXYZ& point) const {
  VoxelKey key;
  key.x = static_cast<int>(std::floor(point.x / voxel_size));
  key.y = static_cast<int>(std::floor(point.y / voxel_size));
  key.z = static_cast<int>(std::floor(point.z / voxel_size));
  return key;
}

// 다운샘플링 함수 구현
pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelGrid::downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
  // 결과 포인트 클라우드
  auto output = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // VoxelKey를 정렬 방식으로 비교하는 map 사용
  std::map<VoxelKey, pcl::PointXYZ, bool (*)(const VoxelKey&, const VoxelKey&)> voxel_map(compareVoxelKeys);

  // 모든 포인트에 대해
  for (const auto& point : *input) {
    // 포인트가 속한 복셀 키 계산
    VoxelKey key = getVoxelKey(point);

    // 이 복셀에 아직 포인트가 없으면 추가
    if (voxel_map.find(key) == voxel_map.end()) {
      voxel_map[key] = point;
    }
  }

  // 결과 포인트 클라우드 구성
  output->reserve(voxel_map.size());
  for (const auto& item : voxel_map) {
    output->push_back(item.second);
  }

  return output;
}