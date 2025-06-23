#include "../include/robust_icp/robust_icp.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <cmath>
#include <iostream>

RobustICP::RobustICP(float scale_parameter, int max_iterations, float convergence_criteria)
    : scale_param_(scale_parameter), max_iterations_(max_iterations), convergence_criteria_(convergence_criteria), final_iterations_(0) {}

void RobustICP::setScaleParameter(float scale) { scale_param_ = scale; }

void RobustICP::setMaxIterations(int max_iter) { max_iterations_ = max_iter; }

void RobustICP::setConvergenceCriteria(float epsilon) { convergence_criteria_ = epsilon; }

int RobustICP::getFinalIterationCount() const { return final_iterations_; }

float RobustICP::gemanMcClureWeight(float error_squared) {
  // Geman-McClure 커널: ρ(e) = e²/2 / (κ + e²)
  // 가중치: w(e) = ρ'(e)/e = κ / (κ + e²)²
  float denominator = scale_param_ + error_squared;
  return scale_param_ / (denominator * denominator);
}

std::vector<std::pair<int, int>> RobustICP::findCorrespondences(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                                                                const Eigen::Matrix4f& transform, float max_distance) {
  // KD-트리를 사용한 최근접 이웃 검색
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(target_cloud);

  std::vector<std::pair<int, int>> correspondences;
  correspondences.reserve(source_cloud->size());

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

    if (kdtree.nearestKSearch(search_point, 1, indices, distances) > 0) {
      // 최대 거리 이내의 대응점만 유지
      if (distances[0] <= max_distance * max_distance) {
        correspondences.push_back(std::make_pair(i, indices[0]));
      }
    }
  }

  return correspondences;
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

  // 특이 값 분해 결과가 반사를 포함할 경우 처리 (행렬식이 1이 아닌 경우)
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

Eigen::Matrix4f RobustICP::align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, float max_correspondence_distance,
                                 const Eigen::Matrix4f& initial_guess) {
  // 현재 변환 초기화
  Eigen::Matrix4f current_transform = initial_guess;
  Eigen::Matrix4f delta_transform;

  // 반복 횟수 초기화
  final_iterations_ = 0;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    // 대응점 찾기
    auto correspondences = findCorrespondences(source_cloud, target_cloud, current_transform, max_correspondence_distance);

    if (correspondences.empty()) {
      std::cout << "경고: 대응점을 찾을 수 없습니다. 현재 변환을 반환합니다." << std::endl;
      break;
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

    // 현재 변환 업데이트
    current_transform = delta_transform * current_transform;

    // 수렴 확인
    float delta_translation = delta_transform.block<3, 1>(0, 3).norm();
    float delta_rotation = Eigen::AngleAxisf(delta_transform.block<3, 3>(0, 0)).angle();

    final_iterations_ = iter + 1;

    if (delta_translation < convergence_criteria_ && delta_rotation < convergence_criteria_) {
      break;
    }
  }

  return current_transform;
}