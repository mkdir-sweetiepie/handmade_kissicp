#include <Eigen/Dense>
#include <cmath>
#include <iomanip>  // setprecision을 위해 추가
#include <iostream>
#include <vector>

#include "../include/evaluation/odometry_evaluator.h"

using namespace evaluation;

int main() {
  std::cout << "======================================" << std::endl;
  std::cout << "    Ground Truth vs 추정 궤적 비교 테스트" << std::endl;
  std::cout << "======================================" << std::endl;

  // 가상의 Ground Truth 생성 (곡선 경로)
  std::vector<TrajectoryPose> ground_truth;
  for (int i = 0; i < 200; ++i) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

    // 원형 경로 생성
    double t = i * 0.1;    // 시간
    double radius = 50.0;  // 반지름 50m

    pose(0, 3) = radius * cos(t * 0.05);  // x
    pose(1, 3) = radius * sin(t * 0.05);  // y
    pose(2, 3) = 0.0;                     // z

    // 방향도 설정
    double yaw = t * 0.05 + M_PI / 2;
    pose(0, 0) = cos(yaw);
    pose(0, 1) = -sin(yaw);
    pose(1, 0) = sin(yaw);
    pose(1, 1) = cos(yaw);

    ground_truth.emplace_back(t, pose);
  }

  // 가상의 추정 궤적 생성 (노이즈 포함)
  std::vector<TrajectoryPose> estimated;
  for (int i = 0; i < 200; ++i) {
    Eigen::Matrix4d pose = ground_truth[i].pose;  // 기본은 GT와 동일

    // 노이즈 추가
    double noise_scale = 0.1 + (i * 0.001);  // 점진적으로 증가하는 오차

    pose(0, 3) += (rand() % 1000 - 500) * 0.001 * noise_scale;   // x 노이즈
    pose(1, 3) += (rand() % 1000 - 500) * 0.001 * noise_scale;   // y 노이즈
    pose(2, 3) += (rand() % 1000 - 500) * 0.0005 * noise_scale;  // z 노이즈

    // 회전 노이즈
    double yaw_noise = (rand() % 1000 - 500) * 0.0001 * noise_scale;
    double original_yaw = atan2(pose(1, 0), pose(0, 0));
    double new_yaw = original_yaw + yaw_noise;

    pose(0, 0) = cos(new_yaw);
    pose(0, 1) = -sin(new_yaw);
    pose(1, 0) = sin(new_yaw);
    pose(1, 1) = cos(new_yaw);

    estimated.emplace_back(ground_truth[i].timestamp, pose);
  }

  std::cout << "테스트 궤적 생성 완료:" << std::endl;
  std::cout << "   Ground Truth: " << ground_truth.size() << " poses (원형 경로)" << std::endl;
  std::cout << "   추정 궤적: " << estimated.size() << " poses (노이즈 포함)" << std::endl;

  // 평가 수행
  std::cout << "\n정확도 평가 실행 중..." << std::endl;

  OdometryEvaluator evaluator;
  evaluator.setGroundTruth(ground_truth);
  evaluator.setEstimatedTrajectory(estimated);

  auto result = evaluator.evaluate();

  // 4결과 출력
  if (result.evaluation_success) {
    std::cout << "\n 평가 결과:" << std::endl;
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

    std::cout << "   ATE (Absolute Trajectory Error):" << std::endl;
    std::cout << "   RMSE: " << std::fixed << std::setprecision(6) << result.ate_rmse << " m" << std::endl;
    std::cout << "   Mean: " << result.ate_mean << " m" << std::endl;
    std::cout << "   Median: " << result.ate_median << " m" << std::endl;
    std::cout << "   Std: " << result.ate_std << " m" << std::endl;

    std::cout << "\n RPE (Relative Pose Error):" << std::endl;
    std::cout << "   Translation RMSE: " << std::setprecision(4) << result.rpe_translation.translation_rmse << " m" << std::endl;
    std::cout << "   Translation Mean: " << result.rpe_translation.translation_mean << " m" << std::endl;
    std::cout << "   Rotation RMSE: " << std::setprecision(6) << result.rpe_rotation.rotation_rmse << " rad" << std::endl;
    std::cout << "   Rotation Mean: " << result.rpe_rotation.rotation_mean << " rad" << std::endl;

    std::cout << "\n 궤적 정보:" << std::endl;
    std::cout << "   총 길이: " << std::setprecision(2) << result.trajectory_length << " m" << std::endl;
    std::cout << "   포즈 개수: " << result.num_poses << std::endl;

    // 성능 해석
    std::cout << "\n 성능 분석:" << std::endl;
    double translation_error_percent = (result.rpe_translation.translation_rmse / result.trajectory_length) * 100.0;
    std::cout << "   Translation Error: " << std::setprecision(3) << translation_error_percent << "%" << std::endl;

    if (result.ate_rmse < 1.0) {
      std::cout << "    매우 정확한 추정!" << std::endl;
    } else if (result.ate_rmse < 5.0) {
      std::cout << "    양호한 정확도" << std::endl;
    } else {
      std::cout << "    정확도 개선 필요" << std::endl;
    }

    // 결과 저장
    std::cout << "\n 결과 저장 중..." << std::endl;

    if (evaluator.saveResults("./test_evaluation_results.txt", result)) {
      std::cout << "    평가 결과: ./test_evaluation_results.txt" << std::endl;
    }

    if (evaluator.saveTrajectoryComparison("./trajectory_comparison.csv")) {
      std::cout << "    궤적 비교: ./trajectory_comparison.csv" << std::endl;
    }

    std::cout << "\n 이것이 바로 Ground Truth와 추정 궤적을 비교하는" << std::endl;
    std::cout << "   정확도 평가 시스템입니다!" << std::endl;

  } else {
    std::cerr << " 평가 실패: " << result.error_message << std::endl;
    return 1;
  }

  std::cout << "\n 테스트 완료!" << std::endl;
  std::cout << "   실제 KITTI 데이터로 평가하려면:" << std::endl;
  std::cout << "   ./complete_kitti_evaluation <kitti_path> <results_path> 00" << std::endl;

  return 0;
}