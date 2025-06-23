#include "../include/constant_velocity/constant_velocity_model.h"

#include <cmath>

// SO(3) 로그 맵 구현 (각속도 계산에 필요)
Eigen::Vector3f logMap(const Eigen::Matrix3f& R) {
  Eigen::AngleAxisf angleAxis(R);
  return angleAxis.angle() * angleAxis.axis();
}

// 생성자
ConstantVelocityModel::ConstantVelocityModel(float dt) : dt_(dt), linear_velocity_(Eigen::Vector3f::Zero()), angular_velocity_(Eigen::Vector3f::Zero()) {}

// 포즈 추가
void ConstantVelocityModel::addPose(const Eigen::Matrix4f& pose) {
  poses_.push_back(pose);

  // 최대 2개의 포즈만 유지
  if (poses_.size() > 2) {
    poses_.pop_front();
  }

  // 두 개의 포즈가 있으면 속도 업데이트
  if (poses_.size() == 2) {
    updateVelocities();
  }
}

// 다음 포즈 예측
Eigen::Matrix4f ConstantVelocityModel::predictNextPose() {
  if (!isInitialized()) {
    // 초기화되지 않았으면 단위 행렬 반환
    return Eigen::Matrix4f::Identity();
  }

  // 등속도 모델에 따른 변환 행렬 계산
  const Eigen::Matrix4f& T_prev = poses_.back();        // T_{t-1}
  const Eigen::Matrix4f& T_prev_prev = poses_.front();  // T_{t-2}

  // 회전 및 이동 추출
  Eigen::Matrix3f R_prev = T_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev = T_prev.block<3, 1>(0, 3);
  Eigen::Matrix3f R_prev_prev = T_prev_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev_prev = T_prev_prev.block<3, 1>(0, 3);

  // 예측 변환 계산: T_pred,t = [R_t-2^T R_t-1, R_t-2^T (t_t-1 - t_t-2); 0 1]
  Eigen::Matrix3f R_pred = R_prev_prev.transpose() * R_prev;
  Eigen::Vector3f t_pred = R_prev_prev.transpose() * (t_prev - t_prev_prev);

  // 결과 변환 행렬 구성
  Eigen::Matrix4f T_pred = Eigen::Matrix4f::Identity();
  T_pred.block<3, 3>(0, 0) = R_pred;
  T_pred.block<3, 1>(0, 3) = t_pred;

  return T_pred;
}

// 속도 업데이트
void ConstantVelocityModel::updateVelocities() {
  if (poses_.size() < 2) return;

  const Eigen::Matrix4f& T_prev = poses_.back();        // T_{t-1}
  const Eigen::Matrix4f& T_prev_prev = poses_.front();  // T_{t-2}

  // 회전 및 이동 추출
  Eigen::Matrix3f R_prev = T_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev = T_prev.block<3, 1>(0, 3);
  Eigen::Matrix3f R_prev_prev = T_prev_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev_prev = T_prev_prev.block<3, 1>(0, 3);

  // 선형 속도: v_t = R_t-2^T (t_t-1 - t_t-2) / Δt
  linear_velocity_ = R_prev_prev.transpose() * (t_prev - t_prev_prev) / dt_;

  // 각속도: w_t = log(R_t-2^T R_t-1) / Δt
  Eigen::Matrix3f R_delta = R_prev_prev.transpose() * R_prev;
  angular_velocity_ = logMap(R_delta) / dt_;
}

// 선형 속도 접근자
Eigen::Vector3f ConstantVelocityModel::getLinearVelocity() const { return linear_velocity_; }

// 각속도 접근자
Eigen::Vector3f ConstantVelocityModel::getAngularVelocity() const { return angular_velocity_; }

// 초기화 여부 확인
bool ConstantVelocityModel::isInitialized() const { return poses_.size() >= 2; }