#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "../include/evaluation/odometry_evaluator.h"

using namespace evaluation;

void testOdometryEvaluator() {
  std::cout << "=== Odometry Evaluator 테스트 ===" << std::endl;

  // Ground truth 궤적 생성
  std::vector<TrajectoryPose> ground_truth;
  for (int i = 0; i < 10; ++i) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = i * 1.0;  // x축으로 1m씩 이동
    double timestamp = i * 0.1;
    ground_truth.emplace_back(timestamp, pose);
  }

  // 추정 궤적 생성 (약간의 오차 포함)
  std::vector<TrajectoryPose> estimated;
  for (int i = 0; i < 10; ++i) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = i * 1.0 + (i * 0.01);  // 작은 누적 오차
    double timestamp = i * 0.1;
    estimated.emplace_back(timestamp, pose);
  }

  OdometryEvaluator evaluator;
  evaluator.setGroundTruth(ground_truth);
  evaluator.setEstimatedTrajectory(estimated);

  auto result = evaluator.evaluate();

  if (result.evaluation_success) {
    std::cout << "평가 성공!" << std::endl;
    std::cout << "ATE RMSE: " << result.ate_rmse << " m" << std::endl;
    std::cout << "ATE Mean: " << result.ate_mean << " m" << std::endl;
    std::cout << "RPE Translation RMSE: " << result.rpe_translation.translation_rmse << " m" << std::endl;
    std::cout << "RPE Rotation RMSE: " << result.rpe_rotation.rotation_rmse << " rad" << std::endl;
    std::cout << "궤적 길이: " << result.trajectory_length << " m" << std::endl;
    std::cout << "포즈 개수: " << result.num_poses << std::endl;
  } else {
    std::cout << "평가 실패: " << result.error_message << std::endl;
  }

  std::cout << "Odometry Evaluator 테스트 완료!" << std::endl;
}

void testRealTimeEvaluation() {
  std::cout << "=== 실시간 평가 테스트 ===" << std::endl;

  std::vector<TrajectoryPose> ground_truth;
  std::vector<TrajectoryPose> estimated;

  for (int i = 0; i < 20; ++i) {
    Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();
    gt_pose(0, 3) = i * 0.5;
    gt_pose(1, 3) = sin(i * 0.1) * 0.2;
    double timestamp = i * 0.1;

    Eigen::Matrix4d est_pose = gt_pose;
    est_pose(0, 3) += (rand() % 100 - 50) * 0.001;
    est_pose(1, 3) += (rand() % 100 - 50) * 0.001;

    ground_truth.emplace_back(timestamp, gt_pose);
    estimated.emplace_back(timestamp, est_pose);

    if ((i + 1) % 10 == 0) {
      OdometryEvaluator temp_evaluator;
      temp_evaluator.setGroundTruth(ground_truth);
      temp_evaluator.setEstimatedTrajectory(estimated);

      auto intermediate_result = temp_evaluator.evaluate();
      if (intermediate_result.evaluation_success) {
        std::cout << "프레임 " << (i + 1) << " - ATE RMSE: " << intermediate_result.ate_rmse << " m" << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "실시간 평가 테스트 완료!" << std::endl;
}

int main() {
  try {
    std::cout << "=========================" << std::endl;
    std::cout << "Evaluation 시스템 테스트 시작" << std::endl;
    std::cout << "=========================" << std::endl << std::endl;

    testOdometryEvaluator();
    testRealTimeEvaluation();

    std::cout << "\n모든 테스트가 성공적으로 완료되었습니다!" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "오류 발생: " << e.what() << std::endl;
    return 1;
  }
}