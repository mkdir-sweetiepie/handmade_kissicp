#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include "../include/evaluation/odometry_evaluator.h"

// 네임스페이스 사용 선언 추가
using namespace evaluation;
// 또는 using evaluation::OdometryEvaluator;

void testOdometryEvaluator() {
  std::cout << "=== Odometry Evaluator 테스트 ===" << std::endl;

  // Ground truth 궤적 생성 (직선 경로)
  std::vector<Eigen::Matrix4f> ground_truth;
  for (int i = 0; i < 10; ++i) {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0, 3) = i * 1.0f;  // x축으로 1m씩 이동
    ground_truth.push_back(pose);
  }

  // 추정 궤적 생성 (약간의 오차 포함)
  std::vector<Eigen::Matrix4f> estimated;
  for (int i = 0; i < 10; ++i) {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0, 3) = i * 1.0f + (i * 0.01f);  // 작은 누적 오차
    estimated.push_back(pose);
  }

  // 이제 네임스페이스 없이 사용 가능
  OdometryEvaluator evaluator;

  // Ground truth 설정
  evaluator.setGroundTruth(ground_truth);

  // 추정 궤적 추가
  for (const auto& pose : estimated) {
    evaluator.addEstimatedPose(pose);
  }

  // KITTI 메트릭 계산
  auto metrics = evaluator.computeKITTIMetrics();

  // 결과 출력
  std::cout << "번역 오차 (상대): " << metrics.avg_translation_error << "%" << std::endl;
  std::cout << "회전 오차 (상대): " << metrics.avg_rotation_error << " deg/m" << std::endl;
  std::cout << "ATE 번역: " << metrics.ate_translation << " m" << std::endl;
  std::cout << "ATE 회전: " << metrics.ate_rotation << " rad" << std::endl;

  std::cout << "Odometry Evaluator 테스트 완료!" << std::endl << std::endl;
}

void testRealTimeEvaluation() {
  std::cout << "=== 실시간 평가 테스트 ===" << std::endl;

  OdometryEvaluator evaluator;

  // 실시간으로 포즈 추가 시뮬레이션
  std::cout << "실시간 odometry 평가 시뮬레이션..." << std::endl;

  for (int i = 0; i < 20; ++i) {
    // Ground truth 포즈 (이상적인 경로)
    Eigen::Matrix4f gt_pose = Eigen::Matrix4f::Identity();
    gt_pose(0, 3) = i * 0.5f;
    gt_pose(1, 3) = sin(i * 0.1f) * 0.2f;  // 약간의 곡선

    // 추정 포즈 (노이즈 포함)
    Eigen::Matrix4f est_pose = gt_pose;
    est_pose(0, 3) += (rand() % 100 - 50) * 0.001f;  // ±5cm 노이즈
    est_pose(1, 3) += (rand() % 100 - 50) * 0.001f;

    // 포즈 추가
    evaluator.addGroundTruthPose(gt_pose);
    evaluator.addEstimatedPose(est_pose);

    // 10프레임마다 중간 결과 출력
    if ((i + 1) % 10 == 0) {
      auto intermediate_metrics = evaluator.computeKITTIMetrics();
      std::cout << "프레임 " << (i + 1) << " - 번역 오차: " 
                << intermediate_metrics.avg_translation_error << "%" << std::endl;
    }

    // 실시간 시뮬레이션을 위한 짧은 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 최종 메트릭 계산
  auto final_metrics = evaluator.computeKITTIMetrics();

  std::cout << "\n=== 최종 평가 결과 ===" << std::endl;
  std::cout << "총 프레임 수: 20" << std::endl;
  std::cout << "평균 번역 오차: " << final_metrics.avg_translation_error << "%" << std::endl;
  std::cout << "평균 회전 오차: " << final_metrics.avg_rotation_error << " deg/m" << std::endl;
  std::cout << "절대 번역 오차 (ATE): " << final_metrics.ate_translation << " m" << std::endl;
  std::cout << "절대 회전 오차 (ATE): " << final_metrics.ate_rotation << " rad" << std::endl;

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