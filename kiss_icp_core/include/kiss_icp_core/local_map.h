#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "odometry_types.h"

// LocalMap 전용 VoxelKey (kiss_icp_voxel과 분리)
struct LocalMapVoxelKey {
  int x, y, z;

  bool operator==(const LocalMapVoxelKey& other) const { 
    return x == other.x && y == other.y && z == other.z; 
  }
};

struct LocalMapVoxelHasher {
  std::size_t operator()(const LocalMapVoxelKey& key) const { 
    return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^ (std::hash<int>()(key.z) << 2); 
  }
};

class LocalMap {
 private:
  std::unordered_map<LocalMapVoxelKey, std::vector<Point3D>, LocalMapVoxelHasher> voxel_map_;
  float voxel_size_;
  int max_points_per_voxel_;
  float max_range_;

  LocalMapVoxelKey getVoxelKey(const Point3D& point) const;

 public:
  LocalMap(float voxel_size = 0.1f, int max_points = 20, float max_range = 100.0f);

  void addPoints(const PointCloud& points, const Eigen::Matrix4f& pose);
  PointCloud getPointsInRange(const Eigen::Vector3f& center, float range);
  void removeDistantVoxels(const Eigen::Vector3f& robot_position);

  size_t size() const;
  void clear();
};