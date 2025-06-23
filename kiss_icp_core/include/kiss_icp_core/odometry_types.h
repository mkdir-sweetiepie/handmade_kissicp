// kiss_icp_core/include/kiss_icp_core/odometry_types.h
#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <vector>

struct Point3D {
  float x, y, z;

  Point3D() : x(0), y(0), z(0) {}
  Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  Eigen::Vector3f toEigen() const { return Eigen::Vector3f(x, y, z); }
  static Point3D fromEigen(const Eigen::Vector3f& v) { return Point3D(v.x(), v.y(), v.z()); }
};

using PointCloud = std::vector<Point3D>;

struct OdometryResult {
  Eigen::Matrix4f pose;
  PointCloud current_scan;
  PointCloud local_map_points;
  std::chrono::high_resolution_clock::time_point timestamp;
  bool success;
  int icp_iterations;
  float adaptive_threshold;

  OdometryResult() : pose(Eigen::Matrix4f::Identity()), success(false), icp_iterations(0), adaptive_threshold(0.0f) {}
};