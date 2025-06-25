#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>

#include "evaluation/odometry_evaluator.h"

namespace fs = std::filesystem;
using namespace evaluation;

class SimpleKITTIEvaluator {
private:
    std::string dataset_path_;
    std::string results_path_;
    OdometryEvaluator evaluator_;
    
public:
    SimpleKITTIEvaluator(const std::string& dataset_path, const std::string& results_path) 
        : dataset_path_(dataset_path), results_path_(results_path) {
        fs::create_directories(results_path_);
        std::cout << "📁 결과 저장 경로: " << results_path_ << std::endl;
    }

    // KITTI Ground Truth 로드 (시뮬레이션)
    std::vector<TrajectoryPose> loadKITTIGroundTruth(const std::string& sequence) {
        std::vector<TrajectoryPose> poses;
        std::string gt_file = dataset_path_ + "/poses/" + sequence + ".txt";

        std::cout << "📖 Ground Truth 파일 시도: " << gt_file << std::endl;

        // 실제 파일이 있으면 로드, 없으면 시뮬레이션 데이터 생성
        std::ifstream file(gt_file);
        if (file.is_open()) {
            std::cout << "✅ 실제 KITTI 파일 발견! 로딩 중..." << std::endl;
            
            std::string line;
            double timestamp = 0.0;
            
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                std::vector<double> values;
                double value;

                while (iss >> value) {
                    values.push_back(value);
                }

                if (values.size() == 12) {
                    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                    pose(0,0) = values[0];  pose(0,1) = values[1];  pose(0,2) = values[2];  pose(0,3) = values[3];
                    pose(1,0) = values[4];  pose(1,1) = values[5];  pose(1,2) = values[6];  pose(1,3) = values[7];
                    pose(2,0) = values[8];  pose(2,1) = values[9];  pose(2,2) = values[10]; pose(2,3) = values[11];
                    
                    poses.emplace_back(timestamp, pose);
                    timestamp += 0.1;
                }
            }
        } else {
            std::cout << "⚠️  실제 KITTI 파일이 없으므로 시뮬레이션 데이터를 생성합니다." << std::endl;
            
            // 시뮬레이션 Ground Truth 생성 (복잡한 경로)
            for (int i = 0; i < 500; ++i) {
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                
                double t = i * 0.1;
                double x = t * 2.0;  // 2m/s로 전진
                double y = 10.0 * sin(t * 0.1);  // 사인파 형태의 횡방향 움직임
                double z = 0.0;
                
                // 방향도 설정
                double yaw = 0.1 * sin(t * 0.05);  // 약간의 방향 변화
                pose(0,0) = cos(yaw);  pose(0,1) = -sin(yaw);
                pose(1,0) = sin(yaw);  pose(1,1) = cos(yaw);
                
                pose(0,3) = x;
                pose(1,3) = y;
                pose(2,3) = z;
                
                poses.emplace_back(t, pose);
            }
        }

        std::cout << "✅ Ground Truth 로드 완료: " << poses.size() << " poses" << std::endl;
        return poses;
    }

    // 가상의 KISS-ICP 결과 생성 (시뮬레이션)
    std::vector<TrajectoryPose> simulateKISSICP(const std::vector<TrajectoryPose>& ground_truth) {
        std::vector<TrajectoryPose> estimated_trajectory;
        estimated_trajectory.reserve(ground_truth.size());

        std::cout << "🤖 KISS-ICP 시뮬레이션 실행 중..." << std::endl;

        for (size_t i = 0; i < ground_truth.size(); ++i) {
            if (i % 50 == 0) {
                std::cout << "   처리 중: " << i+1 << "/" << ground_truth.size() 
                         << " (" << std::fixed << std::setprecision(1) 
                         << (float(i+1)/ground_truth.size())*100 << "%)" << std::endl;
            }

            // Ground Truth에 현실적인 오차 추가
            Eigen::Matrix4d estimated_pose = ground_truth[i].pose;
            
            // 누적 오차 시뮬레이션
            double error_scale = 0.01 + (i * 0.0001);  // 점진적으로 증가하는 오차
            
            // 위치 오차
            estimated_pose(0,3) += (rand() % 1000 - 500) * 0.001 * error_scale;  // x 노이즈
            estimated_pose(1,3) += (rand() % 1000 - 500) * 0.001 * error_scale;  // y 노이즈
            estimated_pose(2,3) += (rand() % 1000 - 500) * 0.0005 * error_scale; // z 노이즈
            
            // 회전 오차
            double yaw_error = (rand() % 1000 - 500) * 0.0001 * error_scale;
            double original_yaw = atan2(estimated_pose(1,0), estimated_pose(0,0));
            double new_yaw = original_yaw + yaw_error;
            
            estimated_pose(0,0) = cos(new_yaw);  estimated_pose(0,1) = -sin(new_yaw);
            estimated_pose(1,0) = sin(new_yaw);  estimated_pose(1,1) = cos(new_yaw);
            
            estimated_trajectory.emplace_back(ground_truth[i].timestamp, estimated_pose);
        }

        return estimated_trajectory;
    }

    // 메인 평가 함수
    void evaluateSequence(const std::string& sequence) {
        std::cout << "\n🚀 ======================================" << std::endl;
        std::cout << "    KITTI 시퀀스 " << sequence << " 평가 시작" << std::endl;
        std::cout << "======================================" << std::endl;

        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            // 1. Ground Truth 로드
            auto ground_truth = loadKITTIGroundTruth(sequence);
            
            // 2. KISS-ICP 결과 시뮬레이션
            auto estimated_trajectory = simulateKISSICP(ground_truth);

            std::cout << "✅ 궤적 처리 완료: " << ground_truth.size() << " poses" << std::endl;

            // 3. 평가 수행
            std::cout << "📊 정확도 평가 중..." << std::endl;
            
            evaluator_.setGroundTruth(ground_truth);
            evaluator_.setEstimatedTrajectory(estimated_trajectory);
            
            auto eval_result = evaluator_.evaluate();

            // 4. 결과 출력
            if (eval_result.evaluation_success) {
                std::cout << "\n🎯 평가 결과:" << std::endl;
                std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
                std::cout << "📈 ATE (Absolute Trajectory Error):" << std::endl;
                std::cout << "   RMSE: " << std::fixed << std::setprecision(6) << eval_result.ate_rmse << " m" << std::endl;
                std::cout << "   Mean: " << eval_result.ate_mean << " m" << std::endl;
                std::cout << "   Median: " << eval_result.ate_median << " m" << std::endl;
                
                std::cout << "\n📉 RPE (Relative Pose Error):" << std::endl;
                std::cout << "   Translation RMSE: " << std::setprecision(4) << eval_result.rpe_translation.translation_rmse << " m" << std::endl;
                std::cout << "   Rotation RMSE: " << std::setprecision(6) << eval_result.rpe_rotation.rotation_rmse << " rad" << std::endl;
                
                std::cout << "\n📏 궤적 정보:" << std::endl;
                std::cout << "   총 길이: " << std::setprecision(2) << eval_result.trajectory_length << " m" << std::endl;
                std::cout << "   포즈 개수: " << eval_result.num_poses << std::endl;
                
                // 성능 비교 (KITTI 벤치마크 기준)
                std::cout << "\n🏆 성능 평가:" << std::endl;
                float translation_error_percent = (eval_result.rpe_translation.translation_rmse / eval_result.trajectory_length) * 100.0f;
                std::cout << "   Translation Error: " << std::setprecision(2) << translation_error_percent << "% (목표: <2.0%)" << std::endl;
                
                if (translation_error_percent < 1.0f) {
                    std::cout << "   🥇 우수한 성능!" << std::endl;
                } else if (translation_error_percent < 2.0f) {
                    std::cout << "   🥈 양호한 성능!" << std::endl;
                } else {
                    std::cout << "   🥉 개선 필요" << std::endl;
                }

            } else {
                std::cerr << "❌ 평가 실패: " << eval_result.error_message << std::endl;
            }

            // 5. 결과 저장
            saveResults(sequence, eval_result, ground_truth, estimated_trajectory);

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
            std::cout << "\n⏱️  총 처리 시간: " << duration.count() << "초" << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "❌ 시퀀스 " << sequence << " 평가 중 심각한 오류: " << e.what() << std::endl;
        }
    }

    // 결과 저장
    void saveResults(const std::string& sequence, const EvaluationResult& result,
                    const std::vector<TrajectoryPose>& gt, const std::vector<TrajectoryPose>& est) {
        std::string seq_dir = results_path_ + "/sequence_" + sequence + "/";
        fs::create_directories(seq_dir);

        // 1. 평가 결과 저장
        std::string results_file = seq_dir + "evaluation_results.txt";
        evaluator_.saveResults(results_file, result);

        // 2. 궤적 비교 데이터 저장
        std::string comparison_file = seq_dir + "trajectory_comparison.csv";
        evaluator_.saveTrajectoryComparison(comparison_file);

        // 3. KITTI 형식으로 추정 궤적 저장
        std::string kitti_file = seq_dir + "estimated_trajectory_kitti.txt";
        std::ofstream kitti_out(kitti_file);
        if (kitti_out.is_open()) {
            for (const auto& pose : est) {
                kitti_out << evaluator_.poseToKittiString(pose.pose) << "\n";
            }
        }

        std::cout << "💾 결과 저장 완료: " << seq_dir << std::endl;
    }
};

int main(int argc, char** argv) {
    std::cout << "🚀 KISS-ICP KITTI 평가 시스템 (시뮬레이션 모드)" << std::endl;
    std::cout << "================================" << std::endl;

    if (argc < 3) {
        std::cout << "사용법: " << argv[0] << " <kitti_dataset_path> <results_path> [sequence...]" << std::endl;
        std::cout << "예시:" << std::endl;
        std::cout << "  " << argv[0] << " /path/to/kitti ./results 00" << std::endl;
        std::cout << "  " << argv[0] << " ./dummy_path ./results 00  # 시뮬레이션 모드" << std::endl;
        return 1;
    }

    std::string dataset_path = argv[1];
    std::string results_path = argv[2];

    // 시퀀스 목록
    std::vector<std::string> sequences;
    if (argc > 3) {
        for (int i = 3; i < argc; ++i) {
            sequences.push_back(argv[i]);
        }
    } else {
        sequences = {"00"};
    }

    std::cout << "📂 데이터셋 경로: " << dataset_path << std::endl;
    std::cout << "📁 결과 저장 경로: " << results_path << std::endl;
    std::cout << "📋 평가할 시퀀스: ";
    for (const auto& seq : sequences) {
        std::cout << seq << " ";
    }
    std::cout << std::endl;

    try {
        SimpleKITTIEvaluator evaluator(dataset_path, results_path);
        
        for (const auto& sequence : sequences) {
            evaluator.evaluateSequence(sequence);
        }

        std::cout << "\n🎉 모든 평가가 성공적으로 완료되었습니다!" << std::endl;
        std::cout << "📊 결과는 다음 경로에서 확인하세요: " << results_path << std::endl;
        std::cout << "\n💡 이것은 KISS-ICP 평가 시스템의 데모입니다." << std::endl;
        std::cout << "   실제 KITTI 데이터가 있으면 로드하고, 없으면 시뮬레이션 데이터를 사용합니다." << std::endl;
        
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "💥 치명적 오류: " << e.what() << std::endl;
        return 1;
    }
}