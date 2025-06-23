#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <iostream>
#include <random>

#include "../include/robust_icp/robust_icp.h"

// 노이즈와 이상치가 있는 포인트 클라우드 생성
pcl::PointCloud<pcl::PointXYZ>::Ptr addNoiseAndOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float noise_std_dev = 0.01f, float outlier_ratio = 0.1f, float outlier_scale = 1.0f) {
  auto noisy_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

  // 난수 생성기 설정
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> noise_dist(0.0f, noise_std_dev);
  std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);
  std::normal_distribution<float> outlier_dist(0.0f, outlier_scale);

  // 모든 포인트에 노이즈 추가
  for (auto& point : noisy_cloud->points) {
    point.x += noise_dist(gen);
    point.y += noise_dist(gen);
    point.z += noise_dist(gen);

    // 일부 포인트를 이상치로 변환
    if (uniform_dist(gen) < outlier_ratio) {
      point.x += outlier_dist(gen);
      point.y += outlier_dist(gen);
      point.z += outlier_dist(gen);
    }
  }

  return noisy_cloud;
}

// 간단한 형상 생성 (여기서는 삼각형 메시 모양)
pcl::PointCloud<pcl::PointXYZ>::Ptr createSimpleShape(int num_points = 1000) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = num_points;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    // 삼각형 표면 위의 랜덤 포인트 생성
    float a = dist(gen);
    float b = dist(gen);
    if (a + b > 1) {
      a = 1 - a;
      b = 1 - b;
    }
    float c = 1 - a - b;

    // 삼각형의 세 꼭지점
    Eigen::Vector3f v1(0, 0, 0);
    Eigen::Vector3f v2(1, 0, 0);
    Eigen::Vector3f v3(0, 1, 0);

    // 무게중심 좌표법으로 삼각형 내 포인트 계산
    Eigen::Vector3f point = a * v1 + b * v2 + c * v3;

    cloud->points[i].x = point(0);
    cloud->points[i].y = point(1);
    cloud->points[i].z = point(2);
  }

  return cloud;
}

int main() {
  try {
    std::cout << "=========================" << std::endl;
    std::cout << "강인한 ICP 테스트 시작" << std::endl;
    std::cout << "=========================" << std::endl << std::endl;

    // 원본 포인트 클라우드 생성 (또는 PCD 파일에서 로드)
    auto source_cloud = createSimpleShape(1000);
    std::cout << "원본 포인트 클라우드 생성: " << source_cloud->size() << " 포인트" << std::endl;

    // 변환 행렬 생성 (회전 + 이동)
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // 약 30도 회전
    transform.rotate(Eigen::AngleAxisf(M_PI / 6, Eigen::Vector3f::UnitZ()));
    // 이동 벡터
    transform.translation() << 0.5, 0.2, 0.1;

    // 타겟 클라우드 생성 (변환 + 노이즈 + 이상치)
    auto target_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*source_cloud, *target_cloud, transform);
    auto noisy_target = addNoiseAndOutliers(target_cloud, 0.01f, 0.2f, 0.5f);

    std::cout << "타겟 포인트 클라우드 생성: " << noisy_target->size() << " 포인트" << std::endl;
    std::cout << "노이즈와 20% 이상치 추가됨" << std::endl << std::endl;

    // 실제 변환 행렬 출력
    std::cout << "실제 변환 행렬:" << std::endl;
    std::cout << transform.matrix() << std::endl << std::endl;

    // 초기 추측값 (약간의 오차 포함)
    Eigen::Affine3f initial_guess = Eigen::Affine3f::Identity();
    initial_guess.rotate(Eigen::AngleAxisf(M_PI / 7, Eigen::Vector3f::UnitZ()));
    initial_guess.translation() << 0.4, 0.3, 0.0;

    std::cout << "초기 추측 변환 행렬:" << std::endl;
    std::cout << initial_guess.matrix() << std::endl << std::endl;

    // Geman-McClure Robust ICP 실행
    RobustICP robust_icp(0.01f, 50, 1e-6f);  // 스케일 파라미터, 최대 반복 횟수, 수렴 기준

    std::cout << "강인한 ICP 실행 중..." << std::endl;
    Eigen::Matrix4f estimated_transform = robust_icp.align(source_cloud, noisy_target, 0.1f, initial_guess.matrix());

    std::cout << "ICP 완료. 반복 횟수: " << robust_icp.getFinalIterationCount() << std::endl << std::endl;

    std::cout << "추정된 변환 행렬:" << std::endl;
    std::cout << estimated_transform << std::endl << std::endl;

    // 결과 변환 적용
    auto aligned_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*source_cloud, *aligned_cloud, estimated_transform);

    // 오차 계산
    Eigen::Matrix4f error_matrix = transform.matrix().inverse() * estimated_transform;
    Eigen::Vector3f error_translation = error_matrix.block<3, 1>(0, 3);
    Eigen::AngleAxisf error_rotation(error_matrix.block<3, 3>(0, 0));

    std::cout << "변환 오차:" << std::endl;
    std::cout << "  이동 오차: " << error_translation.norm() << " 미터" << std::endl;
    std::cout << "  회전 오차: " << error_rotation.angle() * 180.0f / M_PI << " 도" << std::endl;

    // PCD 파일로 저장
    pcl::io::savePCDFileBinary("source_cloud.pcd", *source_cloud);
    pcl::io::savePCDFileBinary("target_cloud.pcd", *noisy_target);
    pcl::io::savePCDFileBinary("aligned_cloud.pcd", *aligned_cloud);

    std::cout << std::endl << "포인트 클라우드가 PCD 파일로 저장되었습니다:" << std::endl;
    std::cout << "  원본 클라우드: source_cloud.pcd" << std::endl;
    std::cout << "  타겟 클라우드: target_cloud.pcd" << std::endl;
    std::cout << "  정합된 클라우드: aligned_cloud.pcd" << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "오류 발생: " << e.what() << std::endl;
    return 1;
  }
}