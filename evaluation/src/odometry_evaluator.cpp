#include "evaluation/odometry_evaluator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace evaluation {

OdometryEvaluator::OdometryEvaluator() {
  // Constructor implementation
}

bool OdometryEvaluator::loadGroundTruth(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Error: Cannot open ground truth file: " << filepath << std::endl;
    return false;
  }

  ground_truth_.clear();
  std::string line;
  double timestamp = 0.0;

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    
    try {
      Eigen::Matrix4d pose = kittiStringToPose(line);
      ground_truth_.emplace_back(timestamp, pose);
      timestamp += 0.1;  // Assume 10Hz data
    } catch (const std::exception& e) {
      std::cerr << "Error parsing line: " << line << std::endl;
      return false;
    }
  }

  std::cout << "Loaded " << ground_truth_.size() << " ground truth poses" << std::endl;
  return !ground_truth_.empty();
}

bool OdometryEvaluator::loadEstimatedTrajectory(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Error: Cannot open estimated trajectory file: " << filepath << std::endl;
    return false;
  }

  estimated_.clear();
  std::string line;
  double timestamp = 0.0;

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    
    try {
      Eigen::Matrix4d pose = kittiStringToPose(line);
      estimated_.emplace_back(timestamp, pose);
      timestamp += 0.1;  // Assume 10Hz data
    } catch (const std::exception& e) {
      std::cerr << "Error parsing line: " << line << std::endl;
      return false;
    }
  }

  std::cout << "Loaded " << estimated_.size() << " estimated poses" << std::endl;
  return !estimated_.empty();
}

void OdometryEvaluator::setGroundTruth(const std::vector<TrajectoryPose>& trajectory) {
  ground_truth_ = trajectory;
}

void OdometryEvaluator::setEstimatedTrajectory(const std::vector<TrajectoryPose>& trajectory) {
  estimated_ = trajectory;
}

EvaluationResult OdometryEvaluator::evaluate(double alignment_length) {
  EvaluationResult result;
  result.evaluation_success = false;

  if (ground_truth_.empty() || estimated_.empty()) {
    result.error_message = "Empty trajectories";
    return result;
  }

  if (ground_truth_.size() != estimated_.size()) {
    result.error_message = "Trajectory size mismatch";
    return result;
  }

  try {
    // Align trajectories
    Eigen::Matrix4d alignment = alignTrajectories();

    // Calculate ATE
    auto ate_stats = calculateATE(alignment);
    result.ate_rmse = ate_stats.translation_rmse;
    result.ate_mean = ate_stats.translation_mean;
    result.ate_median = ate_stats.translation_median;
    result.ate_std = ate_stats.translation_std;

    // Calculate RPE
    auto [rpe_trans, rpe_rot] = calculateRPE(alignment_length);
    result.rpe_translation = rpe_trans;
    result.rpe_rotation = rpe_rot;

    // Calculate trajectory length
    result.trajectory_length = calculateTrajectoryLength(ground_truth_);
    result.num_poses = ground_truth_.size();

    result.evaluation_success = true;

  } catch (const std::exception& e) {
    result.error_message = e.what();
  }

  return result;
}

Eigen::Matrix4d OdometryEvaluator::alignTrajectories() {
  // Simple alignment using first poses
  if (ground_truth_.empty() || estimated_.empty()) {
    return Eigen::Matrix4d::Identity();
  }

  // Use Umeyama algorithm for better alignment
  size_t n = std::min(ground_truth_.size(), estimated_.size());
  
  Eigen::Matrix3Xd gt_points(3, n);
  Eigen::Matrix3Xd est_points(3, n);

  for (size_t i = 0; i < n; ++i) {
    gt_points.col(i) = ground_truth_[i].pose.block<3, 1>(0, 3);
    est_points.col(i) = estimated_[i].pose.block<3, 1>(0, 3);
  }

  // Compute centroids
  Eigen::Vector3d gt_centroid = gt_points.rowwise().mean();
  Eigen::Vector3d est_centroid = est_points.rowwise().mean();

  // Center the points
  gt_points.colwise() -= gt_centroid;
  est_points.colwise() -= est_centroid;

  // Simple translation-only alignment for now
  Eigen::Matrix4d alignment = Eigen::Matrix4d::Identity();
  alignment.block<3, 1>(0, 3) = gt_centroid - est_centroid;

  return alignment;
}

EvaluationResult::RPEStats OdometryEvaluator::calculateATE(const Eigen::Matrix4d& alignment_transform) {
  std::vector<double> translation_errors;
  std::vector<double> rotation_errors;

  for (size_t i = 0; i < std::min(ground_truth_.size(), estimated_.size()); ++i) {
    Eigen::Matrix4d aligned_est = alignment_transform * estimated_[i].pose;
    Eigen::Matrix4d error = ground_truth_[i].pose.inverse() * aligned_est;

    // Translation error
    double trans_error = error.block<3, 1>(0, 3).norm();
    translation_errors.push_back(trans_error);

    // Rotation error (angle of rotation matrix)
    Eigen::Matrix3d R_error = error.block<3, 3>(0, 0);
    double trace = R_error.trace();
    double angle = std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
    rotation_errors.push_back(angle);
  }

  return calculateStatistics(translation_errors);
}

std::pair<EvaluationResult::RPEStats, EvaluationResult::RPEStats> 
OdometryEvaluator::calculateRPE(double distance_threshold) {
  std::vector<double> translation_errors;
  std::vector<double> rotation_errors;

  for (size_t i = 0; i < ground_truth_.size() - 1; ++i) {
    for (size_t j = i + 1; j < ground_truth_.size(); ++j) {
      // Check if distance is approximately distance_threshold
      Eigen::Vector3d gt_diff = ground_truth_[j].pose.block<3, 1>(0, 3) - 
                                ground_truth_[i].pose.block<3, 1>(0, 3);
      double distance = gt_diff.norm();

      if (std::abs(distance - distance_threshold) < 10.0) {  // 10m tolerance
        // Calculate relative poses
        Eigen::Matrix4d gt_rel = ground_truth_[i].pose.inverse() * ground_truth_[j].pose;
        Eigen::Matrix4d est_rel = estimated_[i].pose.inverse() * estimated_[j].pose;

        // Calculate error
        Eigen::Matrix4d error = gt_rel.inverse() * est_rel;

        // Translation error as percentage
        double trans_error = error.block<3, 1>(0, 3).norm() / distance * 100.0;
        translation_errors.push_back(trans_error);

        // Rotation error in rad/m
        Eigen::Matrix3d R_error = error.block<3, 3>(0, 0);
        double trace = R_error.trace();
        double angle = std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
        double rot_error = angle / distance;
        rotation_errors.push_back(rot_error);
      }
    }
  }

  auto trans_stats = calculateStatistics(translation_errors);
  auto rot_stats = calculateStatistics(rotation_errors);

  return {trans_stats, rot_stats};
}

EvaluationResult::RPEStats OdometryEvaluator::calculateStatistics(const std::vector<double>& errors) {
  EvaluationResult::RPEStats stats;

  if (errors.empty()) {
    return stats;
  }

  // Calculate mean
  double sum = std::accumulate(errors.begin(), errors.end(), 0.0);
  stats.translation_mean = sum / errors.size();

  // Calculate variance and RMSE
  double variance = 0.0;
  double rmse_sum = 0.0;
  for (double error : errors) {
    variance += (error - stats.translation_mean) * (error - stats.translation_mean);
    rmse_sum += error * error;
  }

  stats.translation_std = std::sqrt(variance / errors.size());
  stats.translation_rmse = std::sqrt(rmse_sum / errors.size());

  // Calculate median
  std::vector<double> sorted_errors = errors;
  std::sort(sorted_errors.begin(), sorted_errors.end());
  size_t n = sorted_errors.size();
  if (n % 2 == 0) {
    stats.translation_median = (sorted_errors[n/2 - 1] + sorted_errors[n/2]) / 2.0;
  } else {
    stats.translation_median = sorted_errors[n/2];
  }

  return stats;
}

double OdometryEvaluator::calculateTrajectoryLength(const std::vector<TrajectoryPose>& trajectory) {
  double length = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    Eigen::Vector3d diff = trajectory[i].pose.block<3, 1>(0, 3) - 
                          trajectory[i-1].pose.block<3, 1>(0, 3);
    length += diff.norm();
  }
  return length;
}

void OdometryEvaluator::printResults(const EvaluationResult& results) {
  if (!results.evaluation_success) {
    std::cout << "Evaluation failed: " << results.error_message << std::endl;
    return;
  }

  std::cout << "\n========================================" << std::endl;
  std::cout << "        ODOMETRY EVALUATION RESULTS" << std::endl;
  std::cout << "========================================" << std::endl;

  std::cout << std::fixed << std::setprecision(6);
  
  std::cout << "\nTrajectory Info:" << std::endl;
  std::cout << "  Number of poses: " << results.num_poses << std::endl;
  std::cout << "  Trajectory length: " << std::setprecision(2) << results.trajectory_length << " m" << std::endl;

  std::cout << "\nAbsolute Trajectory Error (ATE):" << std::endl;
  std::cout << "  RMSE: " << std::setprecision(6) << results.ate_rmse << " m" << std::endl;
  std::cout << "  Mean: " << results.ate_mean << " m" << std::endl;
  std::cout << "  Median: " << results.ate_median << " m" << std::endl;
  std::cout << "  Std: " << results.ate_std << " m" << std::endl;

  std::cout << "\nRelative Pose Error (RPE) - Translation:" << std::endl;
  std::cout << "  RMSE: " << std::setprecision(4) << results.rpe_translation.translation_rmse << " %" << std::endl;
  std::cout << "  Mean: " << results.rpe_translation.translation_mean << " %" << std::endl;

  std::cout << "\nRelative Pose Error (RPE) - Rotation:" << std::endl;
  std::cout << "  RMSE: " << std::setprecision(6) << results.rpe_rotation.rotation_rmse << " rad/m" << std::endl;
  std::cout << "  Mean: " << results.rpe_rotation.rotation_mean << " rad/m" << std::endl;

  std::cout << "========================================\n" << std::endl;
}

std::string OdometryEvaluator::poseToKittiString(const Eigen::Matrix4d& pose) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(9);
  
  // KITTI format: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ss << pose(i, j);
      if (i != 2 || j != 3) ss << " ";
    }
  }
  
  return ss.str();
}

Eigen::Matrix4d OdometryEvaluator::kittiStringToPose(const std::string& line) {
  std::istringstream iss(line);
  std::vector<double> values;
  double value;
  
  while (iss >> value) {
    values.push_back(value);
  }
  
  if (values.size() != 12) {
    throw std::runtime_error("Invalid KITTI pose format");
  }
  
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  
  // Fill the transformation matrix
  pose(0, 0) = values[0];  pose(0, 1) = values[1];  pose(0, 2) = values[2];  pose(0, 3) = values[3];
  pose(1, 0) = values[4];  pose(1, 1) = values[5];  pose(1, 2) = values[6];  pose(1, 3) = values[7];
  pose(2, 0) = values[8];  pose(2, 1) = values[9];  pose(2, 2) = values[10]; pose(2, 3) = values[11];
  
  return pose;
}

bool OdometryEvaluator::saveResults(const std::string& filepath, const EvaluationResult& results) {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  file << "# KISS-ICP Evaluation Results\n";
  file << "evaluation_success: " << (results.evaluation_success ? "true" : "false") << "\n";
  if (!results.evaluation_success) {
    file << "error_message: " << results.error_message << "\n";
    return true;
  }

  file << "num_poses: " << results.num_poses << "\n";
  file << "trajectory_length: " << results.trajectory_length << "\n";
  file << "ate_rmse: " << results.ate_rmse << "\n";
  file << "ate_mean: " << results.ate_mean << "\n";
  file << "rpe_translation_rmse: " << results.rpe_translation.translation_rmse << "\n";
  file << "rpe_rotation_rmse: " << results.rpe_rotation.rotation_rmse << "\n";

  return true;
}

bool OdometryEvaluator::saveTrajectoryComparison(const std::string& filepath) {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  file << "timestamp,gt_x,gt_y,gt_z,est_x,est_y,est_z\n";
  
  size_t n = std::min(ground_truth_.size(), estimated_.size());
  for (size_t i = 0; i < n; ++i) {
    auto gt_pos = ground_truth_[i].pose.block<3, 1>(0, 3);
    auto est_pos = estimated_[i].pose.block<3, 1>(0, 3);
    
    file << ground_truth_[i].timestamp << ","
         << gt_pos.x() << "," << gt_pos.y() << "," << gt_pos.z() << ","
         << est_pos.x() << "," << est_pos.y() << "," << est_pos.z() << "\n";
  }

  return true;
}

// Placeholder implementations for unused functions
int OdometryEvaluator::findClosestPose(const std::vector<TrajectoryPose>& trajectory, double timestamp) {
  // Simple implementation
  for (size_t i = 0; i < trajectory.size(); ++i) {
    if (std::abs(trajectory[i].timestamp - timestamp) < 0.05) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

TrajectoryPose OdometryEvaluator::interpolatePose(const TrajectoryPose& pose1, const TrajectoryPose& pose2, double t) {
  // Simple linear interpolation for position
  double timestamp = pose1.timestamp + t * (pose2.timestamp - pose1.timestamp);
  Eigen::Matrix4d pose = pose1.pose;
  
  Eigen::Vector3d pos1 = pose1.pose.block<3, 1>(0, 3);
  Eigen::Vector3d pos2 = pose2.pose.block<3, 1>(0, 3);
  pose.block<3, 1>(0, 3) = pos1 + t * (pos2 - pos1);
  
  return TrajectoryPose(timestamp, pose);
}

}  // namespace evaluation