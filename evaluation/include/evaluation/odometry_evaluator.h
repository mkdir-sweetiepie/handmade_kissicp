#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <numeric>

namespace evaluation {

/**
 * @brief Structure to hold trajectory pose data
 */
struct TrajectoryPose {
  double timestamp;
  Eigen::Matrix4d pose;

  TrajectoryPose(double t, const Eigen::Matrix4d& p) : timestamp(t), pose(p) {}
};

/**
 * @brief Structure to hold evaluation results
 */
struct EvaluationResult {
  // Absolute Trajectory Error (ATE)
  double ate_rmse;
  double ate_mean;
  double ate_median;
  double ate_std;
  double ate_min;
  double ate_max;

  // Relative Pose Error (RPE)
  struct RPEStats {
    double translation_rmse;
    double translation_mean;
    double translation_median;
    double translation_std;
    double rotation_rmse;  // in radians
    double rotation_mean;
    double rotation_median;
    double rotation_std;
  };

  RPEStats rpe_translation;  // per 100m
  RPEStats rpe_rotation;     // per 100m

  // Trajectory length
  double trajectory_length;

  // Number of poses
  size_t num_poses;

  // Success flag
  bool evaluation_success;

  // Error message if evaluation failed
  std::string error_message;
};

/**
 * @brief Class for evaluating LiDAR odometry performance
 *
 * This class provides functionality to evaluate odometry algorithms
 * using standard metrics like ATE (Absolute Trajectory Error) and
 * RPE (Relative Pose Error) following KITTI evaluation protocol.
 */
class OdometryEvaluator {
 public:
  /**
   * @brief Constructor
   */
  OdometryEvaluator();

  /**
   * @brief Load ground truth trajectory from file
   * @param filepath Path to ground truth trajectory file (KITTI format)
   * @return True if successful
   */
  bool loadGroundTruth(const std::string& filepath);

  /**
   * @brief Load estimated trajectory from file
   * @param filepath Path to estimated trajectory file (KITTI format)
   * @return True if successful
   */
  bool loadEstimatedTrajectory(const std::string& filepath);

  /**
   * @brief Set ground truth trajectory directly
   * @param trajectory Vector of poses with timestamps
   */
  void setGroundTruth(const std::vector<TrajectoryPose>& trajectory);

  /**
   * @brief Set estimated trajectory directly
   * @param trajectory Vector of poses with timestamps
   */
  void setEstimatedTrajectory(const std::vector<TrajectoryPose>& trajectory);

  /**
   * @brief Evaluate the trajectories using KITTI metrics
   * @param alignment_length Length for trajectory alignment (default: 100m)
   * @return Evaluation results
   */
  EvaluationResult evaluate(double alignment_length = 100.0);

  /**
   * @brief Save evaluation results to file
   * @param filepath Output file path
   * @param results Evaluation results to save
   * @return True if successful
   */
  bool saveResults(const std::string& filepath, const EvaluationResult& results);

  /**
   * @brief Save trajectory comparison plot data
   * @param filepath Output file path (CSV format)
   * @return True if successful
   */
  bool saveTrajectoryComparison(const std::string& filepath);

  /**
   * @brief Print evaluation results to console
   * @param results Evaluation results to print
   */
  void printResults(const EvaluationResult& results);

  /**
   * @brief Convert transformation matrix to KITTI format string
   * @param pose 4x4 transformation matrix
   * @return KITTI format string (12 values)
   */
  static std::string poseToKittiString(const Eigen::Matrix4d& pose);

  /**
   * @brief Parse KITTI format string to transformation matrix
   * @param line KITTI format string
   * @return 4x4 transformation matrix
   */
  static Eigen::Matrix4d kittiStringToPose(const std::string& line);

 private:
  std::vector<TrajectoryPose> ground_truth_;
  std::vector<TrajectoryPose> estimated_;

  /**
   * @brief Align trajectories using Horn's method (Umeyama algorithm)
   * @return Alignment transformation
   */
  Eigen::Matrix4d alignTrajectories();

  /**
   * @brief Calculate absolute trajectory error
   * @param alignment_transform Transformation to align trajectories
   * @return ATE statistics
   */
  EvaluationResult::RPEStats calculateATE(const Eigen::Matrix4d& alignment_transform);

  /**
   * @brief Calculate relative pose error
   * @param distance_threshold Distance threshold for RPE calculation
   * @return RPE statistics
   */
  std::pair<EvaluationResult::RPEStats, EvaluationResult::RPEStats> calculateRPE(double distance_threshold = 100.0);

  /**
   * @brief Calculate statistics from error vector
   * @param errors Vector of error values
   * @return Statistical measures
   */
  EvaluationResult::RPEStats calculateStatistics(const std::vector<double>& errors);

  /**
   * @brief Find closest pose in trajectory by timestamp
   * @param trajectory Trajectory to search in
   * @param timestamp Target timestamp
   * @return Index of closest pose, or -1 if not found
   */
  int findClosestPose(const std::vector<TrajectoryPose>& trajectory, double timestamp);

  /**
   * @brief Calculate trajectory length
   * @param trajectory Input trajectory
   * @return Total length in meters
   */
  double calculateTrajectoryLength(const std::vector<TrajectoryPose>& trajectory);

  /**
   * @brief Interpolate pose between two trajectory poses
   * @param pose1 First pose
   * @param pose2 Second pose
   * @param t Interpolation factor [0,1]
   * @return Interpolated pose
   */
  TrajectoryPose interpolatePose(const TrajectoryPose& pose1, const TrajectoryPose& pose2, double t);
};

}  // namespace evaluation