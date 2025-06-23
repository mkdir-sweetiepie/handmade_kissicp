#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "../include/adaptive_threshold/adaptive_threshold.h"

// 회전 행렬 생성 함수
Eigen::Matrix3f createRotationMatrix(float angle_x, float angle_y, float angle_z) {
  Eigen::AngleAxisf roll_angle(angle_x, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(angle_y, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(angle_z, Eigen::Vector3f::UnitZ());

  Eigen::Quaternion<float> q = yaw_angle * pitch_angle * roll_angle;
  return q.matrix();
}

// 변환 행렬 생성 함수
Eigen::Matrix4f createTransformMatrix(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

int main() {
  try {
    std::cout << "===========================" << std::endl;
    std::cout << "Adaptive Threshold 테스트 시작" << std::endl;
    std::cout << "===========================" << std::endl << std::endl;

    // AdaptiveThreshold 객체 생성 (최대 범위 80m, 최소 변위 0.1m)
    AdaptiveThreshold adaptive_threshold(80.0f, 0.1f);

    std::cout << "초기 임계값: " << adaptive_threshold.getThreshold() << "m" << std::endl << std::endl;

    // 다양한 포즈 변화 시나리오 테스트
    std::cout << "다양한 로봇 움직임 시나리오 테스트:" << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    // 시나리오 1: 작은 움직임
    {
      std::cout << "시나리오 1: 작은 움직임" << std::endl;

      // 작은 회전과 이동
      Eigen::Matrix3f rotation = createRotationMatrix(0.01f, 0.01f, 0.01f);
      Eigen::Vector3f translation(0.05f, 0.05f, 0.02f);
      Eigen::Matrix4f delta_T = createTransformMatrix(rotation, translation);

      float displacement = adaptive_threshold.computeDisplacement(delta_T);
      std::cout << "  예상 변위: " << displacement << "m" << std::endl;

      adaptive_threshold.updateWithPoseDeviation(delta_T);
      std::cout << "  업데이트 후 임계값: " << adaptive_threshold.getThreshold() << "m" << std::endl;
    }
    std::cout << std::endl;

    // 시나리오 2: 중간 크기 움직임
    {
      std::cout << "시나리오 2: 중간 크기 움직임" << std::endl;

      // 중간 정도의 회전과 이동
      Eigen::Matrix3f rotation = createRotationMatrix(0.05f, 0.08f, 0.03f);
      Eigen::Vector3f translation(0.3f, 0.2f, 0.1f);
      Eigen::Matrix4f delta_T = createTransformMatrix(rotation, translation);

      float displacement = adaptive_threshold.computeDisplacement(delta_T);
      std::cout << "  예상 변위: " << displacement << "m" << std::endl;

      adaptive_threshold.updateWithPoseDeviation(delta_T);
      std::cout << "  업데이트 후 임계값: " << adaptive_threshold.getThreshold() << "m" << std::endl;
    }
    std::cout << std::endl;

    // 시나리오 3: 큰 움직임
    {
      std::cout << "시나리오 3: 큰 움직임" << std::endl;

      // 큰 회전과 이동
      Eigen::Matrix3f rotation = createRotationMatrix(0.2f, 0.15f, 0.1f);
      Eigen::Vector3f translation(1.0f, 0.8f, 0.3f);
      Eigen::Matrix4f delta_T = createTransformMatrix(rotation, translation);

      float displacement = adaptive_threshold.computeDisplacement(delta_T);
      std::cout << "  예상 변위: " << displacement << "m" << std::endl;

      adaptive_threshold.updateWithPoseDeviation(delta_T);
      std::cout << "  업데이트 후 임계값: " << adaptive_threshold.getThreshold() << "m" << std::endl;
    }
    std::cout << std::endl;

    // 여러 움직임 후 최종 임계값
    std::cout << "최종 적응형 임계값: " << adaptive_threshold.getThreshold() << "m" << std::endl;
    std::cout << "이 값은 로봇의 움직임 패턴에 따라 자동으로 조정됩니다." << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "오류 발생: " << e.what() << std::endl;
    return 1;
  }
}