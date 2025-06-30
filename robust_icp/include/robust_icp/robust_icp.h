#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <vector>

class RobustICP {
 public:
  // 생성자: 스케일 파라미터와 최대 반복 횟수 설정
  RobustICP(float scale_parameter = 0.05f, int max_iterations = 20, float convergence_criteria = 1e-3f);

  // 강인한 ICP 실행: 두 포인트 클라우드 정합
  Eigen::Matrix4f align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, float max_correspondence_distance,
                        const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity());

  // 스케일 파라미터 설정
  void setScaleParameter(float scale);

  // 반복 횟수 설정
  void setMaxIterations(int max_iter);

  // 수렴 기준 설정
  void setConvergenceCriteria(float epsilon);

  // 최종 반복 횟수 확인
  int getFinalIterationCount() const;

 private:
  float scale_param_;           // 강인 커널의 스케일 파라미터
  int max_iterations_;          // 최대 반복 횟수
  float convergence_criteria_;  // 수렴 기준
  int final_iterations_;        // 최종 반복 횟수

  // 대응점 찾기
  std::vector<std::pair<int, int>> findCorrespondences(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                                                       const Eigen::Matrix4f& transform, float max_distance);
  bool isValidTransformation(const Eigen::Matrix4f& transform);
  // Geman-McClure 커널 가중치 계산
  float gemanMcClureWeight(float error_squared);

  // 가중치를 적용한 포인트-포인트 ICP
  Eigen::Matrix4f estimateRigidTransformation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                                              const std::vector<std::pair<int, int>>& correspondences, const std::vector<float>& weights);
};