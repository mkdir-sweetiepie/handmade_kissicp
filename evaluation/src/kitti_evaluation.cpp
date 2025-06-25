#include <pcl/io/pcd_io.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "../include/evaluation/odometry_evaluator.h"
#include <iomanip>  // setprecision을 위해 추가

namespace fs = std::filesystem;
using namespace evaluation;  // evaluation 네임스페이스 사용

// 누락된 구조체 정의
struct KITTIMetrics {
    float translation_error_percent;
    float rotation_error_deg_per_m;
};

struct RPEResult {
    float mean_translation_error;
    float mean_rotation_error;
    float rmse_translation;
    float rmse_rotation;
};

// 간단한 KISS_ICP 클래스 (실제 구현으로 대체 필요)
class KISS_ICP {
public:
    Eigen::Matrix4f processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 임시 구현 - 실제로는 ICP 알고리즘 수행
        return Eigen::Matrix4f::Identity();
    }
};

class KITTIEvaluator {
 private:
  std::string dataset_path_;
  std::string results_path_;
  OdometryEvaluator evaluator_;

 public:
  KITTIEvaluator(const std::string& dataset_path, const std::string& results_path) 
    : dataset_path_(dataset_path), results_path_(results_path) {
    fs::create_directories(results_path_);
  }

  // KITTI Ground Truth 로드
  std::vector<Eigen::Matrix4f> loadKITTIGroundTruth(const std::string& sequence) {
    std::vector<Eigen::Matrix4f> poses;
    std::string gt_file = dataset_path_ + "/poses/" + sequence + ".txt";

    std::ifstream file(gt_file);
    if (!file.is_open()) {
      throw std::runtime_error("Ground Truth 파일을 열 수 없습니다: " + gt_file);
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::vector<float> values;
      float value;

      while (iss >> value) {
        values.push_back(value);
      }

      if (values.size() == 12) {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose(0, 0) = values[0];  pose(0, 1) = values[1];  pose(0, 2) = values[2];  pose(0, 3) = values[3];
        pose(1, 0) = values[4];  pose(1, 1) = values[5];  pose(1, 2) = values[6];  pose(1, 3) = values[7];
        pose(2, 0) = values[8];  pose(2, 1) = values[9];  pose(2, 2) = values[10]; pose(2, 3) = values[11];
        poses.push_back(pose);
      }
    }

    std::cout << "Ground Truth 로드됨: " << poses.size() << " poses" << std::endl;
    return poses;
  }

  // KITTI Point Cloud 로드
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadKITTIPointClouds(const std::string& sequence) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    std::string velodyne_dir = dataset_path_ + "/sequences/" + sequence + "/velodyne/";

    if (!fs::exists(velodyne_dir)) {
      throw std::runtime_error("Velodyne 디렉토리를 찾을 수 없습니다: " + velodyne_dir);
    }

    // 임시로 빈 클라우드 생성 (실제로는 bin 파일 로드)
    for (int i = 0; i < 100; ++i) {  // 테스트용 100개 프레임
      auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      cloud->width = 1000; cloud->height = 1; cloud->points.resize(1000);
      clouds.push_back(cloud);
    }

    std::cout << "Point Clouds 로드됨: " << clouds.size() << " clouds" << std::endl;
    return clouds;
  }

  // KISS-ICP 실행 및 평가
  void evaluateSequence(const std::string& sequence) {
    std::cout << "\n======================================" << std::endl;
    std::cout << "KITTI 시퀀스 " << sequence << " 평가 시작" << std::endl;
    std::cout << "======================================" << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
      auto ground_truth = loadKITTIGroundTruth(sequence);
      auto point_clouds = loadKITTIPointClouds(sequence);

      if (ground_truth.empty() || point_clouds.empty()) {
        throw std::runtime_error("데이터 로드 실패");
      }

      size_t min_size = std::min(ground_truth.size(), point_clouds.size());
      ground_truth.resize(min_size);
      point_clouds.resize(min_size);

      std::cout << "처리할 프레임 수: " << min_size << std::endl;

      KISS_ICP kiss_icp;
      std::vector<Eigen::Matrix4f> estimated_poses;
      estimated_poses.push_back(Eigen::Matrix4f::Identity());

      std::cout << "KISS-ICP 실행 중..." << std::endl;
      for (size_t i = 1; i < point_clouds.size(); ++i) {
        if (i % 100 == 0) {
          std::cout << "처리 중: " << i << "/" << point_clouds.size() << std::endl;
        }
        auto pose = kiss_icp.processPointCloud(point_clouds[i]);
        estimated_poses.push_back(pose);
      }

      // TrajectoryPose로 변환
      std::vector<evaluation::TrajectoryPose> gt_trajectory, est_trajectory;
      for (size_t i = 0; i < ground_truth.size(); ++i) {
        double timestamp = i * 0.1;
        gt_trajectory.emplace_back(timestamp, ground_truth[i].cast<double>());
        est_trajectory.emplace_back(timestamp, estimated_poses[i].cast<double>());
      }

      evaluator_.setGroundTruth(gt_trajectory);
      evaluator_.setEstimatedTrajectory(est_trajectory);

      auto result = evaluator_.evaluate();
      
      if (result.evaluation_success) {
        std::cout << "\n평가 결과:" << std::endl;
        std::cout << "  ATE RMSE: " << result.ate_rmse << " m" << std::endl;
        std::cout << "  RPE Translation RMSE: " << result.rpe_translation.translation_rmse << " m" << std::endl;
        std::cout << "  RPE Rotation RMSE: " << result.rpe_rotation.rotation_rmse << " rad" << std::endl;
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
      std::cout << "처리 완료. 소요 시간: " << duration.count() << "초" << std::endl;

    } catch (const std::exception& e) {
      std::cerr << "시퀀스 " << sequence << " 평가 중 오류: " << e.what() << std::endl;
    }
  }

  void runFullEvaluation(const std::vector<std::string>& sequences) {
    std::cout << "======================================" << std::endl;
    std::cout << "KISS-ICP KITTI 전체 평가 시작" << std::endl;
    std::cout << "======================================" << std::endl;

    for (const auto& seq : sequences) {
      evaluateSequence(seq);
    }
  }
};

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "사용법: " << argv[0] << " <dataset_path> <results_path> [sequence...]" << std::endl;
    std::cout << "예시: " << argv[0] << " /path/to/kitti ./results 00 01 02" << std::endl;
    return 1;
  }

  std::string dataset_path = argv[1];
  std::string results_path = argv[2];

  std::vector<std::string> sequences;
  if (argc > 3) {
    for (int i = 3; i < argc; ++i) {
      sequences.push_back(argv[i]);
    }
  } else {
    sequences = {"00", "01", "02"};  // 기본 시퀀스
  }

  try {
    KITTIEvaluator evaluator(dataset_path, results_path);
    evaluator.runFullEvaluation(sequences);

    std::cout << "\n모든 평가가 완료되었습니다!" << std::endl;
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "평가 중 오류 발생: " << e.what() << std::endl;
    return 1;
  }
}