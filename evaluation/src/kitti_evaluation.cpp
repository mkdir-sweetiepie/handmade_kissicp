#include <pcl/io/pcd_io.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "../../kiss_icp_core/include/kiss_icp_core/kiss_icp.h"
#include "../include/evaluation/odometry_evaluator.h"

namespace fs = std::filesystem;

class KITTIEvaluator {
 private:
  std::string dataset_path_;
  std::string results_path_;
  OdometryEvaluator evaluator_;

 public:
  KITTIEvaluator(const std::string& dataset_path, const std::string& results_path) : dataset_path_(dataset_path), results_path_(results_path) {
    // 결과 디렉토리 생성
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
        // KITTI 형식: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
        pose(0, 0) = values[0];
        pose(0, 1) = values[1];
        pose(0, 2) = values[2];
        pose(0, 3) = values[3];
        pose(1, 0) = values[4];
        pose(1, 1) = values[5];
        pose(1, 2) = values[6];
        pose(1, 3) = values[7];
        pose(2, 0) = values[8];
        pose(2, 1) = values[9];
        pose(2, 2) = values[10];
        pose(2, 3) = values[11];

        poses.push_back(pose);
      }
    }

    std::cout << "Ground Truth 로드됨: " << poses.size() << " 포즈" << std::endl;
    return poses;
  }

  // KITTI 포인트 클라우드 로드
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadKITTIPointClouds(const std::string& sequence) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    std::string velodyne_path = dataset_path_ + "/sequences/" + sequence + "/velodyne/";

    // 파일 목록 가져오기
    std::vector<std::string> filenames;
    for (const auto& entry : fs::directory_iterator(velodyne_path)) {
      if (entry.path().extension() == ".bin") {
        filenames.push_back(entry.path().filename().string());
      }
    }

    // 파일명으로 정렬
    std::sort(filenames.begin(), filenames.end());

    for (const auto& filename : filenames) {
      std::string full_path = velodyne_path + filename;
      auto cloud = loadKITTIBinary(full_path);
      if (cloud && !cloud->empty()) {
        clouds.push_back(cloud);
      }
    }

    std::cout << "포인트 클라우드 로드됨: " << clouds.size() << " 프레임" << std::endl;
    return clouds;
  }

  // KITTI 바이너리 파일 로드
  pcl::PointCloud<pcl::PointXYZ>::Ptr loadKITTIBinary(const std::string& filepath) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
      std::cerr << "바이너리 파일을 열 수 없습니다: " << filepath << std::endl;
      return cloud;
    }

    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t num_points = file_size / (4 * sizeof(float));  // x, y, z, intensity
    cloud->resize(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      float x, y, z, intensity;
      file.read(reinterpret_cast<char*>(&x), sizeof(float));
      file.read(reinterpret_cast<char*>(&y), sizeof(float));
      file.read(reinterpret_cast<char*>(&z), sizeof(float));
      file.read(reinterpret_cast<char*>(&intensity), sizeof(float));

      cloud->points[i].x = x;
      cloud->points[i].y = y;
      cloud->points[i].z = z;
    }

    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
  }

  // KISS-ICP 실행 및 평가
  void evaluateSequence(const std::string& sequence) {
    std::cout << std::endl << "======================================" << std::endl;
    std::cout << "KITTI 시퀀스 " << sequence << " 평가 시작" << std::endl;
    std::cout << "======================================" << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
      // 데이터 로드
      auto ground_truth = loadKITTIGroundTruth(sequence);
      auto point_clouds = loadKITTIPointClouds(sequence);

      if (ground_truth.empty() || point_clouds.empty()) {
        throw std::runtime_error("데이터 로드 실패");
      }

      // 포인트 클라우드 수와 Ground Truth 수 맞추기
      size_t min_size = std::min(ground_truth.size(), point_clouds.size());
      ground_truth.resize(min_size);
      point_clouds.resize(min_size);

      std::cout << "처리할 프레임 수: " << min_size << std::endl;

      // KISS-ICP 초기화
      KISS_ICP kiss_icp;
      std::vector<Eigen::Matrix4f> estimated_poses;

      // 첫 번째 포즈는 항등행렬
      estimated_poses.push_back(Eigen::Matrix4f::Identity());

      // 각 프레임 처리
      std::cout << "KISS-ICP 실행 중..." << std::endl;
      for (size_t i = 1; i < point_clouds.size(); ++i) {
        if (i % 100 == 0) {
          std::cout << "처리 중: " << i << "/" << point_clouds.size() << std::endl;
        }

        auto pose = kiss_icp.processPointCloud(point_clouds[i]);
        estimated_poses.push_back(pose);
      }

      // 평가 수행
      evaluator_.setGroundTruth(ground_truth);
      evaluator_.setEstimatedTrajectory(estimated_poses);

      // 메트릭 계산
      float ate = evaluator_.computeAbsoluteTrajectoryError();
      auto kitti_metrics = evaluator_.computeKITTIMetrics();

      // RPE 계산 (다양한 거리)
      std::vector<float> distances = {100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f, 700.0f, 800.0f};
      std::vector<RPEResult> rpe_results;
      for (float dist : distances) {
        rpe_results.push_back(evaluator_.computeRelativePoseError(dist));
      }

      // 결과 출력
      std::cout << std::endl << "평가 결과:" << std::endl;
      std::cout << "  ATE: " << std::fixed << std::setprecision(6) << ate << " m" << std::endl;
      std::cout << "  변위 오차: " << std::setprecision(2) << kitti_metrics.translation_error_percent << "%" << std::endl;
      std::cout << "  회전 오차: " << std::setprecision(6) << kitti_metrics.rotation_error_deg_per_m << " deg/m" << std::endl;

      std::cout << std::endl << "RPE 결과:" << std::endl;
      for (size_t i = 0; i < distances.size(); ++i) {
        std::cout << "  " << distances[i] << "m: t_err=" << std::setprecision(4) << rpe_results[i].mean_translation_error << "m, r_err=" << rpe_results[i].mean_rotation_error << "rad" << std::endl;
      }

      // 결과 저장
      std::string result_dir = results_path_ + "/sequence_" + sequence + "/";
      fs::create_directories(result_dir);

      // 궤적 저장
      evaluator_.saveTrajectoryToFile(result_dir + "ground_truth.txt", ground_truth, "TUM");
      evaluator_.saveTrajectoryToFile(result_dir + "estimated.txt", estimated_poses, "TUM");
      evaluator_.saveTrajectoryToFile(result_dir + "estimated_kitti.txt", estimated_poses, "KITTI");

      // 메트릭 저장
      saveMetrics(result_dir + "metrics.txt", sequence, ate, kitti_metrics, rpe_results, distances);

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
      std::cout << std::endl << "처리 완료. 소요 시간: " << duration.count() << "초" << std::endl;

    } catch (const std::exception& e) {
      std::cerr << "시퀀스 " << sequence << " 평가 중 오류: " << e.what() << std::endl;
    }
  }

  // 메트릭을 파일로 저장
  void saveMetrics(const std::string& filepath, const std::string& sequence, float ate, const KITTIMetrics& kitti_metrics, const std::vector<RPEResult>& rpe_results,
                   const std::vector<float>& distances) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
      std::cerr << "메트릭 파일을 열 수 없습니다: " << filepath << std::endl;
      return;
    }

    file << "KISS-ICP Evaluation Results" << std::endl;
    file << "Sequence: " << sequence << std::endl;
    file << "=========================" << std::endl << std::endl;

    file << "ATE (Absolute Trajectory Error): " << std::fixed << std::setprecision(6) << ate << " m" << std::endl;
    file << "Translation Error: " << std::setprecision(2) << kitti_metrics.translation_error_percent << "%" << std::endl;
    file << "Rotation Error: " << std::setprecision(6) << kitti_metrics.rotation_error_deg_per_m << " deg/m" << std::endl;

    file << std::endl << "RPE (Relative Pose Error):" << std::endl;
    file << "Distance(m)\tTrans_Err(m)\tRot_Err(rad)\tTrans_RMSE(m)\tRot_RMSE(rad)" << std::endl;
    for (size_t i = 0; i < distances.size(); ++i) {
      file << distances[i] << "\t" << std::setprecision(4) << rpe_results[i].mean_translation_error << "\t" << rpe_results[i].mean_rotation_error << "\t" << rpe_results[i].rmse_translation << "\t"
           << rpe_results[i].rmse_rotation << std::endl;
    }
  }

  // 전체 평가 실행
  void runFullEvaluation(const std::vector<std::string>& sequences) {
    std::cout << "======================================" << std::endl;
    std::cout << "KISS-ICP KITTI 전체 평가 시작" << std::endl;
    std::cout << "======================================" << std::endl;

    // 전체 결과 수집용
    std::vector<std::string> seq_names;
    std::vector<float> ate_results;
    std::vector<float> trans_errors;
    std::vector<float> rot_errors;

    for (const auto& seq : sequences) {
      evaluateSequence(seq);

      // 결과 수집 (마지막 평가 결과 사용)
      float ate = evaluator_.computeAbsoluteTrajectoryError();
      auto kitti_metrics = evaluator_.computeKITTIMetrics();

      seq_names.push_back(seq);
      ate_results.push_back(ate);
      trans_errors.push_back(kitti_metrics.translation_error_percent);
      rot_errors.push_back(kitti_metrics.rotation_error_deg_per_m);
    }

    // 전체 요약 저장
    saveSummary(seq_names, ate_results, trans_errors, rot_errors);
  }

  // 전체 요약 저장
  void saveSummary(const std::vector<std::string>& sequences, const std::vector<float>& ate_results, const std::vector<float>& trans_errors, const std::vector<float>& rot_errors) {
    std::string summary_file = results_path_ + "/evaluation_summary.txt";
    std::ofstream file(summary_file);

    if (!file.is_open()) {
      std::cerr << "요약 파일을 열 수 없습니다: " << summary_file << std::endl;
      return;
    }

    file << "KISS-ICP KITTI Evaluation Summary" << std::endl;
    file << "=================================" << std::endl << std::endl;

    file << "Sequence\tATE(m)\t\tTrans_Err(%)\tRot_Err(deg/m)" << std::endl;

    float avg_ate = 0.0f, avg_trans = 0.0f, avg_rot = 0.0f;

    for (size_t i = 0; i < sequences.size(); ++i) {
      file << sequences[i] << "\t\t" << std::fixed << std::setprecision(4) << ate_results[i] << "\t\t" << std::setprecision(2) << trans_errors[i] << "\t\t" << std::setprecision(6) << rot_errors[i]
           << std::endl;

      avg_ate += ate_results[i];
      avg_trans += trans_errors[i];
      avg_rot += rot_errors[i];
    }

    avg_ate /= sequences.size();
    avg_trans /= sequences.size();
    avg_rot /= sequences.size();

    file << std::endl << "Average:\t" << std::setprecision(4) << avg_ate << "\t\t" << std::setprecision(2) << avg_trans << "\t\t" << std::setprecision(6) << avg_rot << std::endl;

    std::cout << std::endl << "전체 평가 완료. 요약 파일: " << summary_file << std::endl;
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

  // 시퀀스 목록
  std::vector<std::string> sequences;
  if (argc > 3) {
    for (int i = 3; i < argc; ++i) {
      sequences.push_back(argv[i]);
    }
  } else {
    // 기본 시퀀스 (KITTI Odometry 벤치마크)
    sequences = {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10"};
  }

  try {
    KITTIEvaluator evaluator(dataset_path, results_path);
    evaluator.runFullEvaluation(sequences);

    std::cout << std::endl << "모든 평가가 완료되었습니다!" << std::endl;
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "평가 중 오류 발생: " << e.what() << std::endl;
    return 1;
  }
}