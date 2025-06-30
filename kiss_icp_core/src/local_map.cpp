#include "kiss_icp_core/local_map.h"

#include <iostream>

namespace kiss_icp_core {

LocalMap::LocalMap() { frames_.clear(); }

void LocalMap::addPoints(const PointCloud& points, const Eigen::Matrix4f& pose) {
  MapFrame frame;
  frame.points = points;
  frame.pose = pose;
  frame.timestamp = std::chrono::high_resolution_clock::now();

  frames_.push_back(frame);

  // 최대 프레임 수 제한
  if (frames_.size() > MAX_FRAMES) {
    frames_.pop_front();
  }

  std::cout << "[LocalMap] 프레임 추가됨, 총 프레임 수: " << frames_.size() << std::endl;
}

PointCloud LocalMap::getPointsInRange(const Eigen::Vector3f& center, float max_range) const {
  PointCloud result;

  for (const auto& frame : frames_) {
    // 글로벌 좌표로 변환
    PointCloud transformed = transformPoints(frame.points, frame.pose);

    // 범위 내 포인트만 추가
    for (const auto& point : transformed) {
      Eigen::Vector3f point_pos(point.x, point.y, point.z);
      float distance = (point_pos - center).norm();
      if (distance <= max_range) {
        result.push_back(point);
      }
    }
  }

  return result;
}

PointCloud LocalMap::transformPoints(const PointCloud& points, const Eigen::Matrix4f& transform) const {
  PointCloud transformed;
  transformed.reserve(points.size());

  for (const auto& point : points) {
    Eigen::Vector4f homogeneous(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transform * homogeneous;

    transformed.emplace_back(transformed_point.x(), transformed_point.y(), transformed_point.z());
  }

  return transformed;
}

void LocalMap::clear() { frames_.clear(); }

size_t LocalMap::size() const {
  size_t total_points = 0;
  for (const auto& frame : frames_) {
    total_points += frame.points.size();
  }
  return total_points;
}

}  // namespace kiss_icp_core