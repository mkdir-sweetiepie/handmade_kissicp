#include "../include/adaptive_threshold/adaptive_threshold.h"

#include <cmath>
#include <numeric>

AdaptiveThreshold::AdaptiveThreshold(float max_range, float min_deviation) : r_max_(max_range), delta_min_(min_deviation), current_threshold_(2.0f) {
  // 초기에는 보수적인 임계값 사용
}

float AdaptiveThreshold::getThreshold() const { return current_threshold_; }

void AdaptiveThreshold::updateWithPoseDeviation(const Eigen::Matrix4f& delta_T) {
  // 포즈 변화에 따른 변위 계산
  float displacement = computeDisplacement(delta_T);

  // 유의미한 변위만 히스토리에 추가 (delta_min_ 이상)
  if (displacement > delta_min_) {
    displacements_.push_back(displacement);
    // 임계값 재계산
    recalculateThreshold();
  }
}

float AdaptiveThreshold::computeDisplacement(const Eigen::Matrix4f& delta_T) const {
  // 회전 및 이동 성분 추출
  Eigen::Matrix3f delta_R = delta_T.block<3, 3>(0, 0);
  Eigen::Vector3f delta_t = delta_T.block<3, 1>(0, 3);

  // 회전과 이동 변위 계산
  float rot_displacement = computeRotationalDisplacement(delta_R);
  float trans_displacement = computeTranslationalDisplacement(delta_t);

  // 총 변위 = 회전 변위 + 이동 변위
  return rot_displacement + trans_displacement;
}

float AdaptiveThreshold::computeRotationalDisplacement(const Eigen::Matrix3f& delta_R) const {
  // 회전 행렬에서 회전 각도 추출
  float trace = delta_R.trace();
  float cos_theta = (trace - 1.0f) / 2.0f;
  // 수치 안정성을 위한 범위 제한
  cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
  float theta = std::acos(cos_theta);

  // δ_rot(ΔR) = 2*r_max*sin(θ/2)
  return 2.0f * r_max_ * std::sin(theta / 2.0f);
}

float AdaptiveThreshold::computeTranslationalDisplacement(const Eigen::Vector3f& delta_t) const {
  // δ_trans(Δt) = ||Δt||
  return delta_t.norm();
}

void AdaptiveThreshold::recalculateThreshold() {
  if (displacements_.empty()) {
    return;
  }

  // 표준편차 계산
  float sum = std::accumulate(displacements_.begin(), displacements_.end(), 0.0f);
  float mean = sum / displacements_.size();

  float variance = 0.0f;
  for (const auto& disp : displacements_) {
    variance += (disp - mean) * (disp - mean);
  }
  variance /= displacements_.size();
  float std_dev = std::sqrt(variance);

  // 3-시그마 규칙 적용
  current_threshold_ = 3.0f * std_dev;

  // 최소 유효 임계값 보장
  current_threshold_ = std::max(current_threshold_, delta_min_);
}