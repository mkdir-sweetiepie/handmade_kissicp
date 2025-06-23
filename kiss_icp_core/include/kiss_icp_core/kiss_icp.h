#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <memory>

#include "adaptive_threshold/adaptive_threshold.h"
#include "constant_velocity/constant_velocity_model.h"
#include "kiss_icp_voxel/voxel_grid.h"
#include "local_map.h"
#include "odometry_types.h"
#include "robust_icp/robust_icp.h"
#include "scan_deskewing/scan_deskewer.h"

class KissICP {
 private:
  // 핵심 모듈들
  ConstantVelocityModel velocity_model_;
  ScanDeskewer deskewer_;
  VoxelGrid map_voxel_filter_;  // α=0.5 for map update
  VoxelGrid icp_voxel_filter_;  // β=1.5 for ICP
  AdaptiveThreshold adaptive_threshold_;
  RobustICP robust_icp_;

  // 상태 관리
  LocalMap local_map_;
  Eigen::Matrix4f current_pose_;
  float scan_duration_;
  float max_range_;

  // Helper functions
  PointCloud downsamplePoints(const PointCloud& points, VoxelGrid& filter);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const PointCloud& points);
  PointCloud convertFromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
  Eigen::Matrix4f computePoseDeviation(const Eigen::Matrix4f& predicted, const Eigen::Matrix4f& actual);

 public:
  KissICP();
  ~KissICP() = default;

  // 메인 처리 함수
  OdometryResult processFrame(const PointCloud& raw_scan, const std::vector<float>& timestamps);

  // 설정 함수들
  void setScanDuration(float duration) { scan_duration_ = duration; }
  void setMaxRange(float max_range);

  // 접근자 함수들
  const Eigen::Matrix4f& getCurrentPose() const { return current_pose_; }
  bool isInitialized() const { return velocity_model_.isInitialized(); }

  // 리셋
  void reset();
};