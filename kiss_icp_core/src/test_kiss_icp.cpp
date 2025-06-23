#include <iostream>

#include "kiss_icp_core/kiss_icp.h"

int main() {
  std::cout << "KISS-ICP Core 테스트 시작" << std::endl;

  try {
    KissICP icp;
    std::cout << "KISS-ICP 객체 생성 성공" << std::endl;

    // 간단한 더미 데이터로 테스트
    PointCloud dummy_scan;
    dummy_scan.push_back(Point3D(1.0f, 0.0f, 0.0f));
    dummy_scan.push_back(Point3D(0.0f, 1.0f, 0.0f));
    dummy_scan.push_back(Point3D(0.0f, 0.0f, 1.0f));

    std::vector<float> timestamps = {0.0f, 0.05f, 0.1f};

    auto result = icp.processFrame(dummy_scan, timestamps);

    if (result.success) {
      std::cout << "프레임 처리 성공!" << std::endl;
    } else {
      std::cout << "프레임 처리 실패" << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "오류: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "테스트 완료" << std::endl;
  return 0;
}