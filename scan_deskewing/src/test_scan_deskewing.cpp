#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "../include/scan_deskewing/scan_deskewer.h"

// 벡터 출력 함수
void printVector3f(const std::string& name, const Eigen::Vector3f& vector) {
  std::cout << name << ": [" << std::fixed << std::setprecision(4) << vector.x() << ", " << vector.y() << ", " << vector.z() << "]" << std::endl;
}

// 포인트 클라우드 타입 정의 (KISS-ICP 스타일)
struct PointXYZT {
  PCL_ADD_POINT4D;  // 매크로: XYZ 및 패딩 추가
  float timestamp;  // 타임스탬프 (0~1 사이)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, timestamp, timestamp))

// 테스트용 포인트 클라우드 생성 (원통형)
pcl::PointCloud<PointXYZT>::Ptr createTestCloud(int num_points, float radius, float height) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointXYZT>>();
  cloud->points.reserve(num_points);
  cloud->width = num_points;
  cloud->height = 1;
  cloud->is_dense = true;

  for (int i = 0; i < num_points; ++i) {
    float angle = 2.0f * M_PI * i / num_points;
    float rel_time = static_cast<float>(i) / num_points;  // 0~1 사이의 상대적 시간

    PointXYZT point;
    point.x = radius * std::cos(angle);
    point.y = radius * std::sin(angle);
    point.z = height * rel_time - height / 2;
    point.timestamp = rel_time;

    cloud->points.push_back(point);
  }

  return cloud;
}

// 왜곡된 포인트 클라우드 생성
pcl::PointCloud<pcl::PointXYZ>::Ptr createDistortedCloud(const pcl::PointCloud<PointXYZT>::Ptr& source_cloud, const Eigen::Vector3f& linear_vel, const Eigen::Vector3f& angular_vel,
                                                         float scan_duration) {
  auto distorted_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  distorted_cloud->resize(source_cloud->size());

  for (size_t i = 0; i < source_cloud->size(); ++i) {
    const auto& src_point = source_cloud->points[i];
    float rel_time = src_point.timestamp;

    // 시간에 비례한 변환 계산 (왜곡 모델링)
    Eigen::Vector3f trans = linear_vel * rel_time * scan_duration;
    Eigen::Vector3f rot_axis = angular_vel * rel_time * scan_duration;
    float rot_angle = rot_axis.norm();

    Eigen::Vector3f pos(src_point.x, src_point.y, src_point.z);
    Eigen::Vector3f distorted_pos;

    if (rot_angle > 1e-8) {
      Eigen::AngleAxisf rotation(rot_angle, rot_axis.normalized());
      distorted_pos = rotation * pos - trans;
    } else {
      distorted_pos = pos - trans;
    }

    distorted_cloud->points[i].x = distorted_pos.x();
    distorted_cloud->points[i].y = distorted_pos.y();
    distorted_cloud->points[i].z = distorted_pos.z();
  }

  return distorted_cloud;
}

// 포인트 클라우드 통계 출력
void printCloudStats(const std::string& name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  if (!cloud || cloud->empty()) {
    std::cout << name << ": 비어 있음" << std::endl;
    return;
  }

  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  std::cout << name << " 통계:" << std::endl;
  std::cout << "  포인트 수: " << cloud->size() << std::endl;
  std::cout << "  X 범위: [" << min_pt[0] << ", " << max_pt[0] << "]" << std::endl;
  std::cout << "  Y 범위: [" << min_pt[1] << ", " << max_pt[1] << "]" << std::endl;
  std::cout << "  Z 범위: [" << min_pt[2] << ", " << max_pt[2] << "]" << std::endl;
  std::cout << std::endl;
}

int main() {
  try {
    std::cout << "========================" << std::endl;
    std::cout << "스캔 보정 테스트 시작" << std::endl;
    std::cout << "========================" << std::endl << std::endl;

    // 테스트 파라미터 설정
    Eigen::Vector3f linear_vel(10.0f, 0.0f, 0.0f);   // x방향 10m/s
    Eigen::Vector3f angular_vel(0.0f, 0.52f, 0.0f);  // y축 기준 ~30도/s
    float scan_duration = 0.1f;                      // 스캔에 0.1초 소요

    printVector3f("선형 속도 (m/s)", linear_vel);
    printVector3f("각속도 (rad/s)", angular_vel);
    std::cout << "스캔 기간: " << scan_duration << "초" << std::endl << std::endl;

    // 테스트 포인트 클라우드 생성 (반지름 5m, 높이 2m)
    auto source_cloud = createTestCloud(1000, 5.0f, 2.0f);
    std::cout << "원본 포인트 클라우드 생성: " << source_cloud->size() << " 포인트" << std::endl;

    // 왜곡된 포인트 클라우드 생성
    auto distorted_cloud = createDistortedCloud(source_cloud, linear_vel, angular_vel, scan_duration);
    printCloudStats("왜곡된 포인트 클라우드", distorted_cloud);

    // 타임스탬프 배열 추출
    std::vector<float> timestamps;
    timestamps.reserve(source_cloud->size());
    for (const auto& point : source_cloud->points) {
      timestamps.push_back(point.timestamp);
    }

    // Scan Deskewing 적용
    ScanDeskewer deskewer;
    auto deskewed_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*distorted_cloud);
    deskewer.deskewPointCloud(deskewed_cloud, timestamps, linear_vel, angular_vel, scan_duration);
    printCloudStats("보정된 포인트 클라우드", deskewed_cloud);

    // PCD 파일로 저장
    pcl::io::savePCDFileBinary("distorted_cloud.pcd", *distorted_cloud);
    pcl::io::savePCDFileBinary("deskewed_cloud.pcd", *deskewed_cloud);

    std::cout << "\n포인트 클라우드가 PCD 파일로 저장되었습니다:" << std::endl;
    std::cout << "  왜곡된 클라우드: distorted_cloud.pcd" << std::endl;
    std::cout << "  보정된 클라우드: deskewed_cloud.pcd" << std::endl;
    std::cout << "\n결과를 확인하려면 다음 명령어를 사용하세요:" << std::endl;
    std::cout << "  pcl_viewer deskewed_cloud.pcd" << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "오류 발생: " << e.what() << std::endl;
    return 1;
  }
}