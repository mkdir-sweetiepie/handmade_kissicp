// robust_icp/src/robust_icp.cpp - 근본 해결 버전
#include "../include/robust_icp/robust_icp.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <cmath>
#include <iostream>

RobustICP::RobustICP(float scale_parameter, int max_iterations, float convergence_criteria)
    : scale_param_(scale_parameter), max_iterations_(max_iterations), convergence_criteria_(convergence_criteria), final_iterations_(0) {
  std::cout << "[RobustICP] 초기화 - scale=" << scale_parameter << ", max_iter=" << max_iterations << std::endl;
}

std::vector<std::pair<int, int>> RobustICP::findCorrespondences(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                                                                const Eigen::Matrix4f& transform, float max_distance) {
  std::cout << "[대응점검색] 시작 - Source: " << source_cloud->size() << ", Target: " << target_cloud->size() << ", max_dist: " << max_distance << std::endl;

  if (target_cloud->empty()) {
    std::cout << "[대응점검색] 타겟 클라우드가 비어있음" << std::endl;
    return {};
  }

  // KD-트리 구성
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  try {
    kdtree.setInputCloud(target_cloud);
  } catch (const std::exception& e) {
    std::cerr << "[대응점검색] KD-tree 구성 실패: " << e.what() << std::endl;
    return {};
  }

  std::vector<std::pair<int, int>> correspondences;
  correspondences.reserve(source_cloud->size());

  // 관대한 거리 설정 (기존 문제 해결)
  float expanded_distance = std::max(max_distance * 5.0f, 3.0f);  // 최소 3m
  float distance_squared = expanded_distance * expanded_distance;

  std::cout << "[대응점검색] 확장된 검색 거리: " << expanded_distance << "m" << std::endl;

  int found_count = 0;

  // 각 소스 포인트에 대해 대응점 찾기
  for (size_t i = 0; i < source_cloud->size(); ++i) {
    // 포인트 변환
    Eigen::Vector4f p(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z, 1.0f);
    Eigen::Vector4f p_transformed = transform * p;

    pcl::PointXYZ search_point;
    search_point.x = p_transformed[0];
    search_point.y = p_transformed[1];
    search_point.z = p_transformed[2];

    // K-최근접 이웃 검색 (K=1)
    std::vector<int> indices(1);
    std::vector<float> distances(1);

    try {
      if (kdtree.nearestKSearch(search_point, 1, indices, distances) > 0) {
        // 확장된 거리 내의 대응점 허용
        if (distances[0] <= distance_squared) {
          correspondences.push_back(std::make_pair(i, indices[0]));
          found_count++;
        }
      }
    } catch (const std::exception& e) {
      // 개별 포인트 검색 실패는 무시하고 계속
      continue;
    }

    // 진행 상황 출력 (1000개마다)
    if (i % 1000 == 0 && i > 0) {
      std::cout << "[대응점검색] 진행: " << i << "/" << source_cloud->size() << ", 발견: " << found_count << std::endl;
    }
  }

  float success_rate = (float)found_count / source_cloud->size() * 100.0f;
  std::cout << "[대응점검색] 완료 - 총 " << found_count << "/" << source_cloud->size() << " 대응점 발견 (" << success_rate << "%)" << std::endl;

  // 너무 적은 대응점이면 더 관대하게 재시도
  if (correspondences.size() < 50 && max_distance < 10.0f) {
    std::cout << "[대응점검색] 재시도 - 더 큰 거리로 (" << max_distance * 10.0f << "m)" << std::endl;
    return findCorrespondences(source_cloud, target_cloud, transform, max_distance * 2.0f);
  }

  return correspondences;
}

Eigen::Matrix4f RobustICP::align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, float max_correspondence_distance,
                                 const Eigen::Matrix4f& initial_guess) {
  std::cout << "[ICP정합] 시작 - Source: " << source_cloud->size() << ", Target: " << target_cloud->size() << ", 초기거리: " << max_correspondence_distance << "m" << std::endl;

  // 입력 유효성 검사
  if (source_cloud->empty() || target_cloud->empty()) {
    std::cerr << "[ICP정합] 빈 포인트 클라우드" << std::endl;
    return initial_guess;
  }

  if (source_cloud->size() < 10 || target_cloud->size() < 10) {
    std::cerr << "[ICP정합] 포인트 수 부족" << std::endl;
    return initial_guess;
  }

  // 현재 변환 초기화
  Eigen::Matrix4f current_transform = initial_guess;
  Eigen::Matrix4f delta_transform;

  final_iterations_ = 0;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    std::cout << "[ICP정합] 반복 " << (iter + 1) << "/" << max_iterations_ << std::endl;

    // 대응점 찾기
    auto correspondences = findCorrespondences(source_cloud, target_cloud, current_transform, max_correspondence_distance);

    if (correspondences.empty()) {
      std::cout << "[ICP정합] 경고: 대응점을 찾을 수 없습니다. "
                << "거리=" << max_correspondence_distance << ", Source=" << source_cloud->size() << ", Target=" << target_cloud->size() << std::endl;

      // 거리를 늘려서 재시도
      if (max_correspondence_distance < 20.0f) {
        max_correspondence_distance *= 2.0f;
        std::cout << "[ICP정합] 거리 확장하여 재시도: " << max_correspondence_distance << "m" << std::endl;
        continue;
      } else {
        std::cout << "[ICP정합] 대응점 검색 포기, 현재 변환 반환" << std::endl;
        break;
      }
    }

    std::cout << "[ICP정합] 대응점 " << correspondences.size() << "개 발견" << std::endl;

    // 최소 대응점 수 확인
    if (correspondences.size() < 10) {
      std::cout << "[ICP정합] 대응점 수 부족 (" << correspondences.size() << ")" << std::endl;
      continue;
    }

    // 각 대응점에 대한 오차 계산
    std::vector<float> squared_errors(correspondences.size());
    for (size_t i = 0; i < correspondences.size(); ++i) {
      const auto& source_point = source_cloud->points[correspondences[i].first];
      const auto& target_point = target_cloud->points[correspondences[i].second];

      // 변환된 소스 포인트
      Eigen::Vector4f p_source(source_point.x, source_point.y, source_point.z, 1.0f);
      Eigen::Vector4f p_transformed = current_transform * p_source;

      // 타겟 포인트
      Eigen::Vector3f p_target(target_point.x, target_point.y, target_point.z);

      // 제곱 오차
      squared_errors[i] = (p_transformed.head<3>() - p_target).squaredNorm();
    }

    // Geman-McClure 가중치 계산
    std::vector<float> weights(correspondences.size());
    for (size_t i = 0; i < correspondences.size(); ++i) {
      weights[i] = gemanMcClureWeight(squared_errors[i]);
    }

    // 가중치를 적용한 변환 행렬 추정
    delta_transform = estimateRigidTransformation(source_cloud, target_cloud, correspondences, weights);

    // 변환 유효성 검사
    if (!isValidTransformation(delta_transform)) {
      std::cout << "[ICP정합] 유효하지 않은 변환, 반복 종료" << std::endl;
      break;
    }

    // 현재 변환 업데이트
    current_transform = delta_transform * current_transform;

    // 수렴 확인
    float delta_translation = delta_transform.block<3, 1>(0, 3).norm();
    float delta_rotation = Eigen::AngleAxisf(delta_transform.block<3, 3>(0, 0)).angle();

    std::cout << "[ICP정합] 변화량 - 이동: " << delta_translation << "m, 회전: " << delta_rotation << "rad" << std::endl;

    final_iterations_ = iter + 1;

    if (delta_translation < convergence_criteria_ && delta_rotation < convergence_criteria_) {
      std::cout << "[ICP정합] 수렴 완료 (" << final_iterations_ << " 반복)" << std::endl;
      break;
    }
  }

  std::cout << "[ICP정합] 완료 - " << final_iterations_ << " 반복" << std::endl;
  return current_transform;
}

bool RobustICP::isValidTransformation(const Eigen::Matrix4f& transform) {
  // 회전 행렬이 유효한지 확인 (행렬식이 1이고 직교해야 함)
  Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);

  // 행렬식이 1에 가까운지 확인
  float det = rotation.determinant();
  if (std::abs(det - 1.0f) > 0.1f) {
    return false;
  }

  // 직교 행렬인지 확인 (R * R^T가 단위행렬이어야 함)
  Eigen::Matrix3f should_be_identity = rotation * rotation.transpose();
  Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

  if ((should_be_identity - identity).norm() > 0.1f) {
    return false;
  }

  // 평행이동 값이 합리적인지 확인 (NaN이 아니고 너무 크지 않은지)
  Eigen::Vector3f translation = transform.block<3, 1>(0, 3);
  if (!translation.allFinite() || translation.norm() > 100.0f) {
    return false;
  }

  return true;
}
float RobustICP::gemanMcClureWeight(float error_squared) {
  // Geman-McClure 커널: w(e) = κ / (κ + e²)²
  float denominator = scale_param_ + error_squared;
  return scale_param_ / (denominator * denominator);
}

Eigen::Matrix4f RobustICP::estimateRigidTransformation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                                                       const std::vector<std::pair<int, int>>& correspondences, const std::vector<float>& weights) {
  if (correspondences.empty()) {
    return Eigen::Matrix4f::Identity();
  }

  // 가중 평균 계산
  Eigen::Vector3f source_centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f target_centroid = Eigen::Vector3f::Zero();
  float total_weight = 0.0f;

  for (size_t i = 0; i < correspondences.size(); ++i) {
    const auto& source_point = source_cloud->points[correspondences[i].first];
    const auto& target_point = target_cloud->points[correspondences[i].second];
    float weight = weights[i];

    source_centroid += weight * Eigen::Vector3f(source_point.x, source_point.y, source_point.z);
    target_centroid += weight * Eigen::Vector3f(target_point.x, target_point.y, target_point.z);
    total_weight += weight;
  }

  if (total_weight > 0) {
    source_centroid /= total_weight;
    target_centroid /= total_weight;
  }

  // 공분산 행렬 계산
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

  for (size_t i = 0; i < correspondences.size(); ++i) {
    const auto& source_point = source_cloud->points[correspondences[i].first];
    const auto& target_point = target_cloud->points[correspondences[i].second];
    float weight = weights[i];

    Eigen::Vector3f source_centered = Eigen::Vector3f(source_point.x, source_point.y, source_point.z) - source_centroid;
    Eigen::Vector3f target_centered = Eigen::Vector3f(target_point.x, target_point.y, target_point.z) - target_centroid;

    covariance += weight * target_centered * source_centered.transpose();
  }

  // SVD로 회전 행렬 계산
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();

  // 반사 행렬 보정
  if (rotation.determinant() < 0) {
    Eigen::Matrix3f correction = Eigen::Matrix3f::Identity();
    correction(2, 2) = -1;
    rotation = svd.matrixU() * correction * svd.matrixV().transpose();
  }

  // 이동 벡터 계산
  Eigen::Vector3f translation = target_centroid - rotation * source_centroid;

  // 변환 행렬 구성
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;

  return transform;
}

void RobustICP::setScaleParameter(float scale) {
  scale_param_ = scale;
  std::cout << "[RobustICP] Scale parameter 설정: " << scale << std::endl;
}

void RobustICP::setMaxIterations(int max_iter) {
  max_iterations_ = max_iter;
  std::cout << "[RobustICP] Max iterations 설정: " << max_iter << std::endl;
}

void RobustICP::setConvergenceCriteria(float epsilon) {
  convergence_criteria_ = epsilon;
  std::cout << "[RobustICP] Convergence criteria 설정: " << epsilon << std::endl;
}

int RobustICP::getFinalIterationCount() const { return final_iterations_; }