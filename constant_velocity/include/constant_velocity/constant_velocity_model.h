#pragma once
#include <Eigen/Dense>
#include <deque>

class ConstantVelocityModel {
 public:
  // 생성자: 시간 간격 설정(기본값: 0.1초)
  ConstantVelocityModel(float dt = 0.1f);

  // 이전 포즈를 기반으로 다음 포즈 예측
  Eigen::Matrix4f predictNextPose();

  // 새 포즈 추가
  void addPose(const Eigen::Matrix4f& pose);

  // 선형 속도 및 각속도 접근자
  Eigen::Vector3f getLinearVelocity() const;
  Eigen::Vector3f getAngularVelocity() const;

  // 초기화 여부 확인
  bool isInitialized() const;

 private:
  float dt_;                           // 시간 간격
  std::deque<Eigen::Matrix4f> poses_;  // 포즈 히스토리(최신 2개만 유지)
  Eigen::Vector3f linear_velocity_;    // 선형 속도
  Eigen::Vector3f angular_velocity_;   // 각속도

  // 속도 업데이트
  void updateVelocities();
};