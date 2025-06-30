#pragma once

#include <Eigen/Dense>
#include <deque>

#include "odometry_types.h"

namespace kiss_icp_core {

class LocalMap {
 public:
  LocalMap();

  void addPoints(const PointCloud& points, const Eigen::Matrix4f& pose);
  PointCloud getPointsInRange(const Eigen::Vector3f& center, float max_range) const;
  void clear();
  size_t size() const;

 private:
  struct MapFrame {
    PointCloud points;
    Eigen::Matrix4f pose;
    std::chrono::high_resolution_clock::time_point timestamp;
  };

  std::deque<MapFrame> frames_;
  static constexpr size_t MAX_FRAMES = 20;

  PointCloud transformPoints(const PointCloud& points, const Eigen::Matrix4f& transform) const;
};

}  // namespace kiss_icp_core