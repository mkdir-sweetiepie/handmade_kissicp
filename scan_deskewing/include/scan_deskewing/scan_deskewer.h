#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <vector>

class ScanDeskewer {
 public:
  // 생성자
  ScanDeskewer();

  // 포인트 클라우드 왜곡 보정 (KISS-ICP 스타일의 단일 메서드)
  void deskewPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<float>& timestamps, const Eigen::Vector3f& linear_velocity, const Eigen::Vector3f& angular_velocity,
                        float scan_duration = 0.1f);
};