#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "../include/constant_velocity/constant_velocity_model.h"

// 행렬 출력 함수
void printMatrix4f(const std::string& name, const Eigen::Matrix4f& matrix) {
  std::cout << name << ":\n";
  std::cout << std::fixed << std::setprecision(4);
  for (int i = 0; i < 4; i++) {
    std::cout << "  [";
    for (int j = 0; j < 4; j++) {
      std::cout << std::setw(8) << matrix(i, j);
      if (j < 3) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }
  std::cout << std::endl;
}

// 벡터 출력 함수
void printVector3f(const std::string& name, const Eigen::Vector3f& vector) {
  std::cout << name << ": [" << std::fixed << std::setprecision(4) << vector.x() << ", " << vector.y() << ", " << vector.z() << "]" << std::endl;
}

int main() {
  try {
    std::cout << "========================" << std::endl;
    std::cout << "등속도 모델 테스트 시작" << std::endl;
    std::cout << "========================" << std::endl << std::endl;

    // 등속도 모델 생성 (시간 간격: 0.1초)
    ConstantVelocityModel model(0.1f);

    // 첫 번째 포즈: 원점
    Eigen::Matrix4f pose1 = Eigen::Matrix4f::Identity();
    model.addPose(pose1);
    std::cout << "첫 번째 포즈 추가" << std::endl;
    printMatrix4f("Pose 1", pose1);

    // 이 시점에서는 예측 불가 (포즈가 1개뿐)
    std::cout << "초기화 상태: " << (model.isInitialized() ? "완료" : "미완료") << std::endl << std::endl;

    // 두 번째 포즈: x방향으로 1m 이동, y축 기준 30도 회전
    Eigen::Matrix4f pose2 = Eigen::Matrix4f::Identity();
    float angle = 30.0f * M_PI / 180.0f;  // 30도를 라디안으로 변환
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());
    pose2.block<3, 3>(0, 0) = rotation;
    pose2.block<3, 1>(0, 3) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    model.addPose(pose2);
    std::cout << "두 번째 포즈 추가" << std::endl;
    printMatrix4f("Pose 2", pose2);

    // 이제 예측 가능
    std::cout << "초기화 상태: " << (model.isInitialized() ? "완료" : "미완료") << std::endl << std::endl;

    // 속도 출력
    printVector3f("선형 속도 (m/s)", model.getLinearVelocity());
    printVector3f("각속도 (rad/s)", model.getAngularVelocity());
    std::cout << std::endl;

    // 다음 포즈 예측
    Eigen::Matrix4f predicted_pose = model.predictNextPose();
    std::cout << "다음 포즈 예측" << std::endl;
    printMatrix4f("Predicted Pose", predicted_pose);

    // 예측된 포즈의 의미: 두 번째 포즈에서 예측된 변환을 적용한 결과
    Eigen::Matrix4f next_global_pose = pose2 * predicted_pose;
    std::cout << "전역 좌표계에서의 예측 포즈" << std::endl;
    printMatrix4f("Next Global Pose", next_global_pose);

    // 결과 해석
    std::cout << "결과 해석:" << std::endl;
    std::cout << "- 첫 번째 포즈: 원점, 방향 정면" << std::endl;
    std::cout << "- 두 번째 포즈: x축으로 1m 이동, y축 기준 30도 회전" << std::endl;
    std::cout << "- 예측된 다음 포즈: 이전 움직임을 기반으로 한 다음 위치" << std::endl;
    std::cout << "  (동일한 속도로 계속 이동한다고 가정)" << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "오류 발생: " << e.what() << std::endl;
    return 1;
  }
}