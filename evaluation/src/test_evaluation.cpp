#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include "../include/evaluation/odometry_evaluator.h"

// 네임스페이스 사용 선언 추가
using namespace evaluation;

void testOdometryEvaluator() {
  std::cout << "=== Odometry Evaluator 테스트 ===" << std::endl;

  // Ground truth 궤적 생성 (직선 경로) - TrajectoryPose 형태로 변경
  std::vector<TrajectoryPose> ground_truth;
  for (int i = 0; i < 10; ++i) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = i * 1.0;  // x축으로 1m씩 이동
    double timestamp = i * 0.1;  // 10Hz 데이터 가정
    ground_truth.emplace_back(timestamp, pose);
  }

  // 추정 궤적 생성 (약간의 오차 포함) - TrajectoryPose 형태로 변경
  std::vector<TrajectoryPose> estimated;
  for (int i = 0; i < 10; ++i) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = i * 1.0 + (i * 0.01);  // 작은 누적 오차
    double timestamp = i * 0.1;  // 10Hz 데이터 가정
    estimated.emplace_back(timestamp, pose);
  }

  OdometryEvaluator evaluator;

  // Ground truth 설정 - 올바른 메서드 사용
  evaluator.setGroundTruth(ground_truth);

  // 추정 궤적 설정 - 올바른 메서드 사용
  evaluator.setEstimatedTrajectory(estimated);

  // 평가 수행 - 올바른 메서드 사용
  auto result = evaluator.evaluate();

  // 결과 출력 - 실제 반환되는 구조체 사용
  if (result.evaluation_success) {
    std::cout << "평가 성공!" << std::endl;
    std::cout << "ATE RMSE: " << result.ate_rmse << " m" << std::endl;
    std::cout << "ATE Mean: " << result.ate_mean << " m" << std::endl;
    std::cout << "ATE Median: " << result.ate_median << " m" << std::endl;
    std::cout << "RPE Translation RMSE: " << result.rpe_translation.translation_rmse << " m" << std::endl;
    std::cout << "RPE Rotation RMSE: " << result.rpe_rotation.rotation_rmse << " rad" << std::endl;
    std::cout << "궤적 길이: " << result.trajectory_length << " m" << std::endl;
    std::cout << "포즈 개수: " << result.num_poses << std::endl;
  } else {
    std::cout << "평가 실패: " << result.error_message << std::endl;
  }

  std::cout << "Odometry Evaluator 테스트 완료!" << std::endl << std::endl;
}

void testRealTimeEvaluation() {
  std::cout << "=== 실시간 평가 테스트 ===" << std::endl;

  // 실시간 평가를 위해 궤적을 점진적으로 구축
  std::vector<TrajectoryPose> ground_truth;
  std::vector<TrajectoryPose> estimated;

  std::cout << "실시간 odometry 평가 시뮬레이션..." << std::endl;

  for (int i = 0; i < 20; ++i) {
    // Ground truth 포즈 (이상적인 경로)
    Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();
    gt_pose(0, 3) = i * 0.5;
    gt_pose(1, 3) = sin(i * 0.1) * 0.2;  // 약간의 곡선
    double timestamp = i * 0.1;

    // 추정 포즈 (노이즈 포함)
    Eigen::Matrix4d est_pose = gt_pose;
    est_pose(0, 3) += (rand() % 100 - 50) * 0.001;  // ±5cm 노이즈
    est_pose(1, 3) += (rand() % 100 - 50) * 0.001;

    // 포즈를 벡터에 추가
    ground_truth.emplace_back(timestamp, gt_pose);
    estimated.emplace_back(timestamp, est_pose);

    // 10프레임마다 중간 결과 출력
    if ((i + 1) % 10 == 0) {
      OdometryEvaluator temp_evaluator;
      temp_evaluator.setGroundTruth(ground_truth);
      temp_evaluator.setEstimatedTrajectory(estimated);
      
      auto intermediate_result = temp_evaluator.evaluate();
      if (intermediate_result.evaluation_success) {
        std::cout << "프레임 " << (i + 1) << " - ATE RMSE: " 
                  << intermediate_result.ate_rmse << " m" << std::endl;
      }
    }

    // 실시간 시뮬레이션을 위한 짧은 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 최종 평가
  OdometryEvaluator final_evaluator;
  final_evaluator.setGroundTruth(ground_truth);
  final_evaluator.setEstimatedTrajectory(estimated);
  auto final_result = final_evaluator.evaluate();

  std::cout << "\n=== 최종 평가 결과 ===" << std::endl;
  if (final_result.evaluation_success) {
    std::cout << "총 프레임 수: " << final_result.num_poses << std::endl;
    std::cout << "ATE RMSE: " << final_result.ate_rmse << " m" << std::endl;
    std::cout << "ATE Mean: " << final_result.ate_mean << " m" << std::endl;
    std::cout << "RPE Translation RMSE: " << final_result.rpe_translation.translation_rmse << " m" << std::endl;
    std::cout << "RPE Rotation RMSE: " << final_result.rpe_rotation.rotation_rmse << " rad" << std::endl;
    std::cout << "궤적 길이: " << final_result.trajectory_length << " m" << std::endl;
  } else {
    std::cout << "최종 평가 실패: " << final_result.error_message << std::endl;
  }

  std::cout << "실시간 평가 테스트 완료!" << std::endl;
}

int main() {
  try {
    std::cout << "=========================" << std::endl;
    std::cout << "Evaluation 시스템 테스트 시작" << std::endl;
    std::cout << "=========================" << std::endl << std::endl;

    // 기본 평가기 테스트
    testOdometryEvaluator();

    // 실시간 평가 테스트
    testRealTimeEvaluation();

    std::cout << "\n모든 테스트가 성공적으로 완료되었습니다!" << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "오류 발생: " << e.what() << std::endl;
    return 1;
  }
}