#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <vector>

namespace kiss_icp_core {

// 포인트 타입 정의
struct Point {
  float x, y, z;
  Point() : x(0), y(0), z(0) {}
  Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

using PointCloud = std::vector<Point>;

// 오도메트리 결과
struct OdometryResult {
  bool success;
  Eigen::Matrix4f pose;
  std::chrono::high_resolution_clock::time_point timestamp;

  OdometryResult() : success(false), pose(Eigen::Matrix4f::Identity()) {}
};

}  // namespace kiss_icp_core