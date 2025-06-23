// kiss_icp_core/src/kiss_icp.cpp
#include "kiss_icp_core/kiss_icp.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

KissICP::KissICP()
    : velocity_model_(0.1f)  // 0.1초 간격
      ,
      map_voxel_filter_(0.5f)  // α = 0.5
      ,
      icp_voxel_filter_(1.5f)  // β = 1.5
      ,
      adaptive_threshold_(100.0f, 0.1f)  // max_range, min_deviation
      ,
      robust_icp_(0.01f, 50, 1e-4f)  // scale_param, max_iter, convergence
      ,
      current_pose_(Eigen::Matrix4f::Identity()),
      scan_duration_(0.1f),
      max_range_(100.0f) {}

OdometryResult KissICP::processFrame(const PointCloud& raw_scan, const std::vector<float>& timestamps) {
  OdometryResult result;
  result.timestamp = std::chrono::high_resolution_clock::now();

  if (raw_scan.empty()) {
    result.success = false;
    return result;
  }

  // 1. Motion prediction
  Eigen::Matrix4f predicted_transform = Eigen::Matrix4f::Identity();
  if (velocity_model_.isInitialized()) {
    predicted_transform = velocity_model_.predictNextPose();
  }

  // 2. Scan deskewing
  PointCloud deskewed_scan = raw_scan;
  if (velocity_model_.isInitialized() && !timestamps.empty()) {
    auto linear_vel = velocity_model_.getLinearVelocity();
    auto angular_vel = velocity_model_.getAngularVelocity();

    // Convert to PCL format for deskewing
    auto pcl_cloud = convertToPCL(raw_scan);
    deskewer_.deskewPointCloud(pcl_cloud, timestamps, linear_vel, angular_vel, scan_duration_);
    deskewed_scan = convertFromPCL(pcl_cloud);
  }

  // 3. Double downsampling
  auto map_scan = downsamplePoints(deskewed_scan, map_voxel_filter_);
  auto icp_scan = downsamplePoints(map_scan, icp_voxel_filter_);

  // 4. ICP registration if we have local map
  Eigen::Matrix4f icp_transform = Eigen::Matrix4f::Identity();
  if (local_map_.size() > 0) {
    float threshold = adaptive_threshold_.getThreshold();
    auto map_points = local_map_.getPointsInRange(current_pose_.block<3, 1>(0, 3), max_range_);

    if (!map_points.empty()) {
      auto source_pcl = convertToPCL(icp_scan);
      auto target_pcl = convertToPCL(map_points);

      icp_transform = robust_icp_.align(source_pcl, target_pcl, threshold, predicted_transform);
      result.icp_iterations = robust_icp_.getFinalIterationCount();
    }
  }

  // 5. Update pose and local map
  current_pose_ = icp_transform * current_pose_ * predicted_transform;
  local_map_.addPoints(map_scan, current_pose_);

  // 6. Update models
  velocity_model_.addPose(current_pose_);

  if (velocity_model_.isInitialized()) {
    Eigen::Matrix4f pose_deviation = computePoseDeviation(predicted_transform, icp_transform);
    adaptive_threshold_.updateWithPoseDeviation(pose_deviation);
  }

  // 7. Prepare result
  result.pose = current_pose_;
  result.current_scan = icp_scan;
  result.local_map_points = local_map_.getPointsInRange(current_pose_.block<3, 1>(0, 3), 50.0f);
  result.success = true;
  result.adaptive_threshold = adaptive_threshold_.getThreshold();

  return result;
}

PointCloud KissICP::downsamplePoints(const PointCloud& points, VoxelGrid& filter) {
  auto pcl_cloud = convertToPCL(points);
  auto downsampled = filter.downsample(pcl_cloud);
  return convertFromPCL(downsampled);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KissICP::convertToPCL(const PointCloud& points) {
  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl_cloud->points.reserve(points.size());

  for (const auto& pt : points) {
    pcl_cloud->points.emplace_back(pt.x, pt.y, pt.z);
  }

  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = true;

  return pcl_cloud;
}

PointCloud KissICP::convertFromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
  PointCloud points;
  points.reserve(pcl_cloud->points.size());

  for (const auto& pt : pcl_cloud->points) {
    points.emplace_back(pt.x, pt.y, pt.z);
  }

  return points;
}

Eigen::Matrix4f KissICP::computePoseDeviation(const Eigen::Matrix4f& predicted, const Eigen::Matrix4f& actual) { return predicted.inverse() * actual; }

void KissICP::setMaxRange(float max_range) {
  max_range_ = max_range;
  adaptive_threshold_ = AdaptiveThreshold(max_range, 0.1f);
}

void KissICP::reset() {
  velocity_model_ = ConstantVelocityModel(0.1f);
  adaptive_threshold_ = AdaptiveThreshold(max_range_, 0.1f);
  local_map_.clear();
  current_pose_ = Eigen::Matrix4f::Identity();
}