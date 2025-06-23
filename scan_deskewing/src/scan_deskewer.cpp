#include "../include/scan_deskewing/scan_deskewer.h"

#include <cmath>

ScanDeskewer::ScanDeskewer() {}

void ScanDeskewer::deskewPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<float>& timestamps, const Eigen::Vector3f& linear_velocity, const Eigen::Vector3f& angular_velocity,
                                    float scan_duration) {
  // 포인트 클라우드가 비어있거나 타임스탬프 크기가 맞지 않으면 처리하지 않음
  if (cloud->empty() || cloud->size() != timestamps.size()) {
    return;
  }

  // 각 포인트에 대해 왜곡 보정 적용
  for (size_t i = 0; i < cloud->size(); ++i) {
    // 상대적 타임스탬프 (0~1 사이)
    float relative_time = timestamps[i];

    // 원본 포인트 좌표
    Eigen::Vector3f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    // 회전 계산: Exp(s_i * ω_t)
    float angle = angular_velocity.norm() * relative_time * scan_duration;
    Eigen::Matrix3f rotation;

    if (angle < 1e-8) {
      // 각도가 매우 작으면 회전 없음
      rotation = Eigen::Matrix3f::Identity();
    } else {
      // 회전축 정규화
      Eigen::Vector3f axis = angular_velocity.normalized();

      // Rodrigues 공식을 사용한 회전 행렬 계산
      Eigen::Matrix3f K;
      K << 0, -axis(2), axis(1), axis(2), 0, -axis(0), -axis(1), axis(0), 0;

      rotation = Eigen::Matrix3f::Identity() + std::sin(angle) * K + (1 - std::cos(angle)) * K * K;
    }

    // 보정 적용: p_i* = Exp(s_i * ω_t) * p_i + s_i * v_t
    Eigen::Vector3f deskewed_point = rotation * point + linear_velocity * relative_time * scan_duration;

    // 보정된 값으로 포인트 업데이트
    cloud->points[i].x = deskewed_point(0);
    cloud->points[i].y = deskewed_point(1);
    cloud->points[i].z = deskewed_point(2);
  }
}