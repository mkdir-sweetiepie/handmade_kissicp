#include "../include/constant_velocity/constant_velocity_model.h"

#include <cmath>
// SO(3) 로그 맵 구현 (각속도 계산에 필요) (회전 행렬 -> 각속도 벡터 변환)
Eigen::Vector3f logMap(const Eigen::Matrix3f& R) {
  Eigen::AngleAxisf angleAxis(R);               // 회전 행렬을 각도와 축으로 변환(theta, axis)
  return angleAxis.angle() * angleAxis.axis();  // theta * axis 벡터 반환
}

// 생성자
ConstantVelocityModel::ConstantVelocityModel(float dt) : dt_(dt), linear_velocity_(Eigen::Vector3f::Zero()), angular_velocity_(Eigen::Vector3f::Zero()) {}

// 포즈 추가
void ConstantVelocityModel::addPose(const Eigen::Matrix4f& pose) {
  // pusk_back()을 사용하여 새 포즈 추가 (리스트 제일 뒤에 원소 추가)
  poses_.push_back(pose);

  // 최대 2개의 포즈만 유지
  // pop_front()을 사용하여 가장 오래된 포즈 제거 (리스트 제일 앞에 윈소 제거)
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
  // 4x4 변환 행렬에서 회전과 평행이동을 추출
  Eigen::Matrix3f R_prev = T_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev = T_prev.block<3, 1>(0, 3);
  Eigen::Matrix3f R_prev_prev = T_prev_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev_prev = T_prev_prev.block<3, 1>(0, 3);

  // 예측 변환 계산: T_pred,t = [R_t-2^T R_t-1, R_t-2^T (t_t-1 - t_t-2); 0 1]
  Eigen::Matrix3f R_pred = R_prev_prev.transpose() * R_prev;
  Eigen::Vector3f t_pred = R_prev_prev.transpose() * (t_prev - t_prev_prev);

  // 결과 변환 행렬 구성
  // 4x4 변환 행렬을 구성
  Eigen::Matrix4f T_pred = Eigen::Matrix4f::Identity();
  T_pred.block<3, 3>(0, 0) = R_pred;
  T_pred.block<3, 1>(0, 3) = t_pred;

  return T_pred;
}

// 속도 업데이트
void ConstantVelocityModel::updateVelocities() {
  if (poses_.size() < 2) return;

  // front() 첫번째 원소 반황 (가장 오래된 포즈)
  // back() 마지막 원소 반환 (가장 최근 포즈)
  const Eigen::Matrix4f& T_prev = poses_.back();        // T_{t-1}
  const Eigen::Matrix4f& T_prev_prev = poses_.front();  // T_{t-2}

  // 회전 및 이동 추출
  // block<3, 3>(0, 0) : <> 행렬 크기 3x3, () 시작 행 0, 시작 열 0
  // block<3, 1>(0, 3) : <> 행렬 크기 3x1, () 시작 행 0, 시작 열 3
  Eigen::Matrix3f R_prev = T_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev = T_prev.block<3, 1>(0, 3);
  Eigen::Matrix3f R_prev_prev = T_prev_prev.block<3, 3>(0, 0);
  Eigen::Vector3f t_prev_prev = T_prev_prev.block<3, 1>(0, 3);

  // 선형 속도: v_t = R_t-2^T (t_t-1 - t_t-2) / Δt
  linear_velocity_ = R_prev_prev.transpose() * (t_prev - t_prev_prev) / dt_;

  // 각속도: w_t = log(R_t-2^T R_t-1) / Δt
  Eigen::Matrix3f R_delta = R_prev_prev.transpose() * R_prev;
  angular_velocity_ = logMap(R_delta) / dt_;  // 초당 각속도
}

// 선형 속도 접근자
Eigen::Vector3f ConstantVelocityModel::getLinearVelocity() const { return linear_velocity_; }

// 각속도 접근자
Eigen::Vector3f ConstantVelocityModel::getAngularVelocity() const { return angular_velocity_; }

// 초기화 여부 확인
bool ConstantVelocityModel::isInitialized() const { return poses_.size() >= 2; }