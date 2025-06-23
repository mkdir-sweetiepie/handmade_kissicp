#include "kiss_icp_core/local_map.h"

#include <cmath>

LocalMap::LocalMap(float voxel_size, int max_points, float max_range) : voxel_size_(voxel_size), max_points_per_voxel_(max_points), max_range_(max_range) {}

LocalMapVoxelKey LocalMap::getVoxelKey(const Point3D& point) const {
  return {static_cast<int>(std::floor(point.x / voxel_size_)), static_cast<int>(std::floor(point.y / voxel_size_)), static_cast<int>(std::floor(point.z / voxel_size_))};
}

void LocalMap::addPoints(const PointCloud& points, const Eigen::Matrix4f& pose) {
  for (const auto& point : points) {
    // Transform point to global frame
    Eigen::Vector4f global_point = pose * Eigen::Vector4f(point.x, point.y, point.z, 1.0f);
    Point3D global_pt(global_point.x(), global_point.y(), global_point.z());

    LocalMapVoxelKey key = getVoxelKey(global_pt);

    // Add point if voxel isn't saturated
    auto& voxel_points = voxel_map_[key];
    if (voxel_points.size() < max_points_per_voxel_) {
      voxel_points.push_back(global_pt);
    }
  }
}

PointCloud LocalMap::getPointsInRange(const Eigen::Vector3f& center, float range) {
  PointCloud result;

  for (const auto& [key, points] : voxel_map_) {
    for (const auto& point : points) {
      float distance = (point.toEigen() - center).norm();
      if (distance <= range) {
        result.push_back(point);
      }
    }
  }

  return result;
}

void LocalMap::removeDistantVoxels(const Eigen::Vector3f& robot_position) {
  auto it = voxel_map_.begin();
  while (it != voxel_map_.end()) {
    // Check if any point in this voxel is within range
    bool has_close_point = false;
    for (const auto& point : it->second) {
      float distance = (point.toEigen() - robot_position).norm();
      if (distance <= max_range_) {
        has_close_point = true;
        break;
      }
    }

    if (!has_close_point) {
      it = voxel_map_.erase(it);
    } else {
      ++it;
    }
  }
}

size_t LocalMap::size() const {
  size_t total = 0;
  for (const auto& [key, points] : voxel_map_) {
    total += points.size();
  }
  return total;
}

void LocalMap::clear() { voxel_map_.clear(); }