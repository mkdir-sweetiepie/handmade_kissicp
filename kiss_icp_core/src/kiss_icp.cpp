#include "kiss_icp_core/kiss_icp.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

namespace kiss_icp_core {

KissICP::KissICP()
    : velocity_model_(0.1f),
      map_voxel_filter_(0.5f),  // α = 0.5
      icp_voxel_filter_(1.5f),  // β = 1.5
      adaptive_threshold_(100.0f, 0.1f),
      robust_icp_(0.01f, 50, 1e-4f),
      current_pose_(Eigen::Matrix4f::Identity()),
      scan_duration_(0.1f),
      max_range_(100.0f) {
  std::cout << "[KISS-ICP] 초기화 완료 - 5가지 핵심 모듈 로드됨" << std::endl;
}

void KissICP::setMaxRange(float max_range) {
  max_range_ = max_range;
  adaptive_threshold_ = AdaptiveThreshold(max_range, 0.1f);

  // Voxel 크기를 max_range에 비례하여 조정
  float voxel_size = std::max(0.1f, max_range * 0.01f);
  map_voxel_filter_ = VoxelGrid(voxel_size * 0.5f);  // α = 0.5
  icp_voxel_filter_ = VoxelGrid(voxel_size * 1.5f);  // β = 1.5

  std::cout << "[KISS-ICP] max_range=" << max_range << ", voxel_size=" << voxel_size << std::endl;
}

OdometryResult KissICP::processFrame(const PointCloud& raw_scan, const std::vector<float>& timestamps) {
  OdometryResult result;
  result.timestamp = std::chrono::high_resolution_clock::now();
  result.success = false;

  if (raw_scan.empty()) {
    std::cerr << "[KISS-ICP] 빈 포인트 클라우드 입력" << std::endl;
    return result;
  }

  std::cout << "[KISS-ICP] 프레임 처리 시작 - 포인트 수: " << raw_scan.size() << std::endl;

  // 1. 거리 필터링
  PointCloud filtered_scan = filterByDistance(raw_scan);
  std::cout << "[KISS-ICP] 거리 필터링: " << raw_scan.size() << " -> " << filtered_scan.size() << std::endl;

  if (filtered_scan.size() < 100) {
    std::cerr << "[KISS-ICP] 필터링 후 포인트 부족: " << filtered_scan.size() << std::endl;
    return result;
  }

  // 2. 첫 번째 프레임 처리
  if (!velocity_model_.isInitialized()) {
    return initializeFirstFrame(filtered_scan);
  }

  // 3. Constant Velocity Model - 초기 예측
  Eigen::Matrix4f predicted_pose = velocity_model_.predictNextPose();
  std::cout << "[KISS-ICP] 예측된 이동: [" << predicted_pose(0, 3) << ", " << predicted_pose(1, 3) << ", " << predicted_pose(2, 3) << "]" << std::endl;

  // 4. Scan Deskewing - 모션 왜곡 보정
  PointCloud deskewed_scan = filtered_scan;
  if (velocity_model_.isInitialized() && !timestamps.empty()) {
    auto linear_vel = velocity_model_.getLinearVelocity();
    auto angular_vel = velocity_model_.getAngularVelocity();
    auto pcl_cloud = convertToPCL(filtered_scan);
    deskewer_.deskewPointCloud(pcl_cloud, timestamps, linear_vel, angular_vel, scan_duration_);
    deskewed_scan = convertFromPCL(pcl_cloud);
    std::cout << "[KISS-ICP] Scan Deskewing 완료" << std::endl;
  }

  // 5. Voxel Grid - ICP용 다운샘플링 (β = 1.5)
  PointCloud downsampled_scan = downsamplePoints(deskewed_scan, icp_voxel_filter_);
  std::cout << "[KISS-ICP] ICP 다운샘플링: " << deskewed_scan.size() << " -> " << downsampled_scan.size() << std::endl;

  // 6. Local Map에서 포인트 가져오기
  Eigen::Vector3f robot_pos(current_pose_(0, 3), current_pose_(1, 3), current_pose_(2, 3));
  PointCloud local_map_points = local_map_.getPointsInRange(robot_pos, max_range_);
  std::cout << "[KISS-ICP] Local Map 포인트 수: " << local_map_points.size() << std::endl;

  // 7. Adaptive Threshold - 대응점 검색 임계값 계산
  float threshold = adaptive_threshold_.getThreshold();
  robust_icp_.setScaleParameter(threshold);  // 수정: setThreshold -> setScaleParameter
  std::cout << "[KISS-ICP] Adaptive Threshold: " << threshold << std::endl;

  // 8. Robust ICP - Geman-McClure 커널 사용
  OdometryResult icp_result = performICP(downsampled_scan, local_map_points, predicted_pose);

  if (icp_result.success) {
    // 9. 포즈 업데이트
    Eigen::Matrix4f pose_deviation = computePoseDeviation(predicted_pose, icp_result.pose);
    adaptive_threshold_.updateWithPoseDeviation(pose_deviation);

    current_pose_ = icp_result.pose;
    velocity_model_.addPose(current_pose_);  // 수정: updatePose -> addPose

    // 10. Local Map 업데이트 (α = 0.5로 다운샘플링)
    PointCloud merged_scan = downsamplePoints(deskewed_scan, map_voxel_filter_);
    local_map_.addPoints(merged_scan, current_pose_);

    result.success = true;
    result.pose = current_pose_;

    std::cout << "[KISS-ICP] 성공! 현재 위치: [" << current_pose_(0, 3) << ", " << current_pose_(1, 3) << ", " << current_pose_(2, 3) << "]" << std::endl;
  } else {
    std::cerr << "[KISS-ICP] ICP 등록 실패" << std::endl;
  }

  previous_scan_ = deskewed_scan;
  return result;
}

PointCloud KissICP::filterByDistance(const PointCloud& points) {
  PointCloud filtered;
  filtered.reserve(points.size());

  for (const auto& point : points) {
    float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (distance >= 5.0f && distance <= max_range_) {
      filtered.push_back(point);
    }
  }

  return filtered;
}

OdometryResult KissICP::initializeFirstFrame(const PointCloud& scan) {
  OdometryResult result;
  result.success = true;
  result.pose = current_pose_;
  result.timestamp = std::chrono::high_resolution_clock::now();

  // 첫 번째 프레임을 맵에 추가
  PointCloud downsampled = downsamplePoints(scan, map_voxel_filter_);
  local_map_.addPoints(downsampled, current_pose_);

  // 등속도 모델 초기화
  velocity_model_.addPose(current_pose_);  // 수정: initialize -> addPose

  previous_scan_ = scan;

  std::cout << "[KISS-ICP] 첫 번째 프레임으로 초기화 완료: " << downsampled.size() << " 포인트" << std::endl;
  return result;
}

OdometryResult KissICP::performICP(const PointCloud& source, const PointCloud& target, const Eigen::Matrix4f& initial_guess) {
  OdometryResult result;

  if (target.empty()) {
    std::cerr << "[KISS-ICP] 타겟 포인트클라우드가 비어있음" << std::endl;
    return result;
  }

  // PCL 포인트클라우드로 변환
  auto source_pcl = convertToPCL(source);
  auto target_pcl = convertToPCL(target);

  // Robust ICP 실행 (수정된 시그니처)
  float max_correspondence_distance = adaptive_threshold_.getThreshold();
  Eigen::Matrix4f transformation = robust_icp_.align(source_pcl, target_pcl, max_correspondence_distance, initial_guess);

  if (isValidTransform(transformation)) {
    result.success = true;
    result.pose = transformation;
    std::cout << "[KISS-ICP] Robust ICP 수렴" << std::endl;
  } else {
    std::cerr << "[KISS-ICP] Robust ICP 실패 또는 무효한 변환" << std::endl;
  }

  return result;
}

bool KissICP::isValidTransform(const Eigen::Matrix4f& transform) {
  // 변환 행렬 유효성 검사
  float translation_norm = transform.block<3, 1>(0, 3).norm();
  if (translation_norm > 10.0f) {  // 10m 이상 점프하면 무효
    return false;
  }

  // 회전 행렬 검사
  Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  float det = rotation.determinant();
  if (std::abs(det - 1.0f) > 0.1f) {  // 행렬식이 1에서 너무 멀면 무효
    return false;
  }

  return true;
}

PointCloud KissICP::downsamplePoints(const PointCloud& points, VoxelGrid& filter) {
  // PCL 포인트클라우드로 변환 후 다운샘플링
  auto pcl_cloud = convertToPCL(points);
  auto downsampled_pcl = filter.downsample(pcl_cloud);
  return convertFromPCL(downsampled_pcl);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KissICP::convertToPCL(const PointCloud& points) {
  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl_cloud->points.reserve(points.size());

  for (const auto& point : points) {
    pcl_cloud->points.emplace_back(point.x, point.y, point.z);
  }

  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = false;

  return pcl_cloud;
}

PointCloud KissICP::convertFromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
  PointCloud points;
  points.reserve(pcl_cloud->points.size());

  for (const auto& pcl_point : pcl_cloud->points) {
    points.emplace_back(pcl_point.x, pcl_point.y, pcl_point.z);
  }

  return points;
}

Eigen::Matrix4f KissICP::computePoseDeviation(const Eigen::Matrix4f& predicted, const Eigen::Matrix4f& actual) { return predicted.inverse() * actual; }

void KissICP::reset() {
  current_pose_ = Eigen::Matrix4f::Identity();
  local_map_.clear();
  velocity_model_ = ConstantVelocityModel(0.1f);
  adaptive_threshold_ = AdaptiveThreshold(max_range_, 0.1f);
  previous_scan_.clear();

  std::cout << "[KISS-ICP] 리셋 완료" << std::endl;
}

}  // namespace kiss_icp_core