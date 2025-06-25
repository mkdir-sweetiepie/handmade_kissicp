// kiss_icp_core/src/kiss_icp.cpp - 수정된 버전
#include "kiss_icp_core/kiss_icp.h"

#include <iostream>

KissICP::KissICP()
    : velocity_model_(0.1f),
      map_voxel_filter_(0.5f),
      icp_voxel_filter_(1.5f),
      adaptive_threshold_(100.0f, 0.1f),
      robust_icp_(0.01f, 50, 1e-4f),
      current_pose_(Eigen::Matrix4f::Identity()),
      scan_duration_(0.1f),
      max_range_(100.0f) {
  std::cout << "[KISS-ICP] 초기화 완료" << std::endl;
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

  std::cout << "[KISS-ICP] 프레임 처리 시작, 포인트 수: " << raw_scan.size() << std::endl;

  // 1. 거리 필터링
  PointCloud filtered_scan = filterByDistance(raw_scan);
  std::cout << "[KISS-ICP] 거리 필터링: " << raw_scan.size() << " -> " << filtered_scan.size() << std::endl;

  if (filtered_scan.size() < 100) {
    std::cerr << "[KISS-ICP] 필터링 후 포인트 부족: " << filtered_scan.size() << std::endl;
    return result;
  }

  // 2. 첫 번째 프레임 처리 (특별 처리)
  if (!velocity_model_.isInitialized()) {
    return initializeFirstFrame(filtered_scan);
  }

  // 3. 등속도 모델로 초기 예측
  Eigen::Matrix4f predicted_pose = velocity_model_.predictNextPose();
  std::cout << "[KISS-ICP] 예측된 이동: [" << predicted_pose(0, 3) << ", " << predicted_pose(1, 3) << ", " << predicted_pose(2, 3) << "]" << std::endl;

  // 4. Scan Deskewing
  PointCloud deskewed_scan = filtered_scan;
  if (velocity_model_.isInitialized() && !timestamps.empty()) {
    auto linear_vel = velocity_model_.getLinearVelocity();
    auto angular_vel = velocity_model_.getAngularVelocity();
    auto pcl_cloud = convertToPCL(filtered_scan);
    deskewer_.deskewPointCloud(pcl_cloud, timestamps, linear_vel, angular_vel, scan_duration_);
    deskewed_scan = convertFromPCL(pcl_cloud);
  }

  // 5. ICP용 다운샘플링
  PointCloud downsampled_scan = downsamplePoints(deskewed_scan, icp_voxel_filter_);
  std::cout << "[KISS-ICP] ICP 다운샘플링: " << deskewed_scan.size() << " -> " << downsampled_scan.size() << std::endl;

  // 6. Local Map 포인트 가져오기
  Eigen::Vector3f robot_pos(current_pose_(0, 3), current_pose_(1, 3), current_pose_(2, 3));
  PointCloud local_map_points = local_map_.getPointsInRange(robot_pos, max_range_);
  std::cout << "[KISS-ICP] Local Map 포인트 수: " << local_map_points.size() << std::endl;

  // 7. Local Map이 너무 적으면 이전 스캔 사용
  if (local_map_points.size() < 500) {
    std::cout << "[KISS-ICP] Local Map 부족, 이전 스캔 추가" << std::endl;
    if (!previous_scan_.empty()) {
      local_map_points.insert(local_map_points.end(), previous_scan_.begin(), previous_scan_.end());
    }
  }

  // 8. ICP 수행
  if (local_map_points.size() >= 100) {
    result = performICP(downsampled_scan, local_map_points, predicted_pose);
  } else {
    std::cout << "[KISS-ICP] 타겟 포인트 부족, 예측값 사용" << std::endl;
    result.success = true;
    result.pose = predicted_pose * current_pose_;
  }

  // 9. 성공적으로 처리된 경우
  if (result.success) {
    // 포즈 업데이트
    current_pose_ = result.pose;

    // 등속도 모델 업데이트
    velocity_model_.addPose(current_pose_);

    // Adaptive threshold 업데이트
    if (velocity_model_.isInitialized()) {
      Eigen::Matrix4f pose_deviation = computePoseDeviation(predicted_pose, result.pose);
      adaptive_threshold_.updateWithPoseDeviation(pose_deviation);
    }

    // Local Map 업데이트 (Map용 다운샘플링)
    PointCloud map_points = downsamplePoints(deskewed_scan, map_voxel_filter_);
    local_map_.addPoints(map_points, current_pose_);

    // 이전 스캔 저장
    previous_scan_ = downsampled_scan;

    // 결과 설정
    result.local_map_points = local_map_points;
    result.current_scan = downsampled_scan;
    result.adaptive_threshold = adaptive_threshold_.getThreshold();

    std::cout << "[KISS-ICP] 프레임 처리 성공, 현재 위치: [" << current_pose_(0, 3) << ", " << current_pose_(1, 3) << ", " << current_pose_(2, 3) << "]" << std::endl;
  }

  return result;
}

OdometryResult KissICP::initializeFirstFrame(const PointCloud& scan) {
  std::cout << "[KISS-ICP] 첫 번째 프레임 초기화" << std::endl;

  OdometryResult result;
  result.timestamp = std::chrono::high_resolution_clock::now();
  result.success = true;
  result.pose = current_pose_;  // Identity

  // 첫 번째 스캔을 Local Map에 추가
  PointCloud map_points = downsamplePoints(scan, map_voxel_filter_);
  local_map_.addPoints(map_points, current_pose_);

  // 이전 스캔으로 저장
  previous_scan_ = downsamplePoints(scan, icp_voxel_filter_);

  // 등속도 모델 초기화 (첫 번째 포즈 추가)
  velocity_model_.addPose(current_pose_);

  // 결과 설정
  result.local_map_points = map_points;
  result.current_scan = previous_scan_;

  std::cout << "[KISS-ICP] 첫 번째 프레임 초기화 완료, Local Map: " << map_points.size() << " 포인트" << std::endl;

  return result;
}

PointCloud KissICP::filterByDistance(const PointCloud& points) {
  PointCloud filtered;
  filtered.reserve(points.size());

  for (const auto& point : points) {
    float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    // 최소 거리 1m, 최대 거리 max_range_
    if (distance >= 1.0f && distance <= max_range_) {
      filtered.push_back(point);
    }
  }

  return filtered;
}

OdometryResult KissICP::performICP(const PointCloud& source, const PointCloud& target, const Eigen::Matrix4f& initial_guess) {
  OdometryResult result;
  result.timestamp = std::chrono::high_resolution_clock::now();
  result.success = false;

  // PCL 포인트 클라우드로 변환
  auto source_pcl = convertToPCL(source);
  auto target_pcl = convertToPCL(target);

  if (source_pcl->empty() || target_pcl->empty()) {
    std::cerr << "[ICP] 소스 또는 타겟이 비어있음" << std::endl;
    return result;
  }

  std::cout << "[ICP] 시작 - Source: " << source_pcl->size() << ", Target: " << target_pcl->size() << std::endl;

  // Adaptive Threshold 계산
  float correspondence_distance = adaptive_threshold_.getThreshold();
  if (correspondence_distance < 1.0f) {
    correspondence_distance = 5.0f;  // 최소 5m로 설정
    std::cout << "[ICP] Adaptive Threshold 부족, 기본값 사용: " << correspondence_distance << "m" << std::endl;
  }

  // ICP 수행
  try {
    Eigen::Matrix4f icp_result = robust_icp_.align(source_pcl, target_pcl, correspondence_distance, initial_guess);

    int iterations = robust_icp_.getFinalIterationCount();
    std::cout << "[ICP] 완료 - 반복횟수: " << iterations << ", 거리임계값: " << correspondence_distance << "m" << std::endl;

    // 유효성 체크
    if (isValidTransform(icp_result)) {
      result.success = true;
      result.pose = icp_result * current_pose_;
      result.icp_iterations = iterations;

    } else {
      std::cerr << "[ICP] 유효하지 않은 변환 결과" << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "[ICP] 예외 발생: " << e.what() << std::endl;
  }

  return result;
}

bool KissICP::isValidTransform(const Eigen::Matrix4f& transform) {
  // NaN 및 무한대 체크
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (!std::isfinite(transform(i, j))) {
        return false;
      }
    }
  }

  // 회전 행렬 유효성 체크
  Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  float det = rotation.determinant();
  if (std::abs(det - 1.0f) > 0.1f) {
    std::cerr << "[유효성검사] 회전 행렬 행렬식 이상: " << det << std::endl;
    return false;
  }

  // 직교 행렬인지 확인
  Eigen::Matrix3f should_be_identity = rotation * rotation.transpose();
  Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

  if ((should_be_identity - identity).norm() > 0.1f) {
    return false;
  }

  // 이동 거리 체크 (너무 큰 점프 방지)
  Eigen::Vector3f translation = transform.block<3, 1>(0, 3);
  float movement = translation.norm();
  if (movement > 10.0f) {  // 10m 이상 이동 시 의심
    std::cerr << "[유효성검사] 과도한 이동: " << movement << "m" << std::endl;
    return false;
  }

  return true;
}

PointCloud KissICP::downsamplePoints(const PointCloud& points, VoxelGrid& filter) {
  auto pcl_cloud = convertToPCL(points);
  auto downsampled_pcl = filter.downsample(pcl_cloud);
  return convertFromPCL(downsampled_pcl);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KissICP::convertToPCL(const PointCloud& points) {
  auto pcl_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl_cloud->points.reserve(points.size());

  for (const auto& point : points) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_cloud->points.push_back(pcl_point);
  }

  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = true;

  return pcl_cloud;
}

PointCloud KissICP::convertFromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
  PointCloud points;
  points.reserve(pcl_cloud->size());

  for (const auto& pcl_point : pcl_cloud->points) {
    Point3D point;
    point.x = pcl_point.x;
    point.y = pcl_point.y;
    point.z = pcl_point.z;
    points.push_back(point);
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