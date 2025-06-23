#pragma once
#include <Eigen/Dense>
#include <deque>
#include <vector>

class AdaptiveThreshold {
 public:
  // 생성자: LiDAR 최대 범위와 최소 변위 임계값 설정
  AdaptiveThreshold(float max_range = 100.0f, float min_deviation = 0.1f);

  // 현재 임계값 반환
  float getThreshold() const;

  // 새로운 포즈 변화 추가 후 임계값 업데이트
  void updateWithPoseDeviation(const Eigen::Matrix4f& delta_T);

  // 변환 행렬로부터 예상 변위 계산
  float computeDisplacement(const Eigen::Matrix4f& delta_T) const;

 private:
  float r_max_;                       // LiDAR 최대 측정 범위
  float delta_min_;                   // 최소 변위 임계값
  float current_threshold_;           // 현재 임계값
  std::vector<float> displacements_;  // 유의미한 변위들의 히스토리

  // 회전으로 인한 변위 계산
  float computeRotationalDisplacement(const Eigen::Matrix3f& delta_R) const;

  // 이동으로 인한 변위 계산
  float computeTranslationalDisplacement(const Eigen::Vector3f& delta_t) const;

  // 현재 히스토리로 새 임계값 계산
  void recalculateThreshold();
};