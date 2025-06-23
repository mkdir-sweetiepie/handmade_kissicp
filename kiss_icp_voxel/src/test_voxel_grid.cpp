#include <pcl/io/pcd_io.h>

#include <cstdlib>
#include <ctime>
#include <iostream>

#include "../include/kiss_icp_voxel/voxel_grid.h"

// 랜덤 포인트 클라우드 생성 함수
pcl::PointCloud<pcl::PointXYZ>::Ptr createRandomPointCloud(int n_points) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = n_points;
  cloud->height = 1;
  cloud->points.resize(n_points);

  // 랜덤 시드 초기화
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  // 랜덤 포인트 생성
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    cloud->points[i].y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    cloud->points[i].z = static_cast<float>(rand()) / RAND_MAX * 10.0f;
  }

  return cloud;
}

int main(int argc, char** argv) {
  try {
    std::cout << "========================" << std::endl;
    std::cout << "Voxel Grid 필터 테스트 시작" << std::endl;
    std::cout << "========================" << std::endl << std::endl;
    
    // 랜덤 포인트 클라우드 생성
    auto cloud = createRandomPointCloud(10000);

    // PCD 파일이 인자로 제공된 경우 로딩
    if (argc > 1) {
      cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        std::cerr << "PCD 파일을 로드할 수 없습니다: " << argv[1] << std::endl;
        return -1;
      }
      std::cout << "파일에서 로드된 포인트 수: " << cloud->size() << std::endl;
    }

    // Voxel Grid 필터 적용
    float voxel_size = 0.5f;  // 기본 복셀 크기

    // 명령행 인자로 복셀 크기 지정 가능
    if (argc > 2) {
      voxel_size = std::stof(argv[2]);
    }

    std::cout << "복셀 크기: " << voxel_size << std::endl;

    VoxelGrid filter(voxel_size);
    auto filtered_cloud = filter.downsample(cloud);

    // 결과 출력
    std::cout << "원본 포인트 수: " << cloud->size() << std::endl;
    std::cout << "필터링 후 포인트 수: " << filtered_cloud->size() << std::endl;
    double compression_ratio = (1.0 - static_cast<double>(filtered_cloud->size()) / cloud->size()) * 100.0;
    std::cout << "압축률: " << compression_ratio << "%" << std::endl;

    // 결과 PCD 파일로 저장
    std::string original_filename = "original_cloud.pcd";
    std::string filtered_filename = "filtered_cloud.pcd";

    pcl::io::savePCDFileASCII(original_filename, *cloud);
    pcl::io::savePCDFileASCII(filtered_filename, *filtered_cloud);

    std::cout << "원본 포인트 클라우드 저장됨: " << original_filename << std::endl;
    std::cout << "필터링된 포인트 클라우드 저장됨: " << filtered_filename << std::endl;
    std::cout << "결과를 보려면 다음 명령어 사용: pcl_viewer " << filtered_filename << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "오류 발생: " << e.what() << std::endl;
    return 1;
  }
}