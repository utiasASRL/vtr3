#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <numeric>
#include <random>
#include <unordered_set>

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Dense>

#include <lgmath.hpp>
#include <steam.hpp>

#include <vtr_lidar/cloud/cloud.h>
#include <vtr_lidar/pointmap/pointmap.hpp>
#include <vtr_logging/logging.hpp>

namespace vtr {
namespace lidar {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

struct ICPQuery {
  ICPQuery(const std::vector<double>& t, const std::vector<PointXYZ>& p,
           const std::vector<float>& w)
      : time(t), points(p), weights(w) {}
  const std::vector<double>& time;
  const std::vector<PointXYZ>& points;
  const std::vector<float>& weights;
};

struct ICPParams {
  // number of threads for nearest neighbor search
  int num_threads = 1;

  // number of points sampled at each step
  size_t n_samples = 1000;

  // Pairing distance threshold
  float max_pairing_dist = 5.0;
  float max_planar_dist = 0.3;

  // Convergence thresholds
  size_t max_iter = 50;
  size_t avg_steps = 3;
  float rot_diff_thresh = 0.1 * M_PI / 180.0;  // threshold on variation of R
  float trans_diff_thresh = 0.01;              // threshold on variation of T

  // For motion distortion, angle phi of the last transform
  float init_phi = 0.0;
  bool motion_distortion = false;

  // Initial transformation
  Eigen::Matrix4d init_transform = Eigen::Matrix4d::Identity(4, 4);
};

struct ICPResults {
  // Final transformation
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity(4, 4);
  Eigen::MatrixXd all_transforms = Eigen::MatrixXd::Zero(4, 4);

  // Final RMS error
  std::vector<float> all_rms = std::vector<float>();
  std::vector<float> all_plane_rms = std::vector<float>();

  // Covariance
  Matrix6d covariance = Matrix6d::Zero();

  // Points performance
  // double intersection_over_union = 0;
  double matched_points_ratio = 0;
};

Matrix6d computeCovariance(
    const Eigen::Matrix4d& T,
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>& targets,
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>& references,
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>& ref_normals,
    const std::vector<float>& weights,
    const std::vector<std::pair<size_t, size_t>>& sample_inds);

void pointToMapICP(ICPQuery& query, PointMap& map, ICPParams& params,
                   ICPResults& results);

}  // namespace lidar
}  // namespace vtr

std::ostream& operator<<(std::ostream& os, const vtr::lidar::ICPParams& s);
