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

#include "vtr_lidar/cloud/cloud.h"
#include "vtr_lidar/pointmap/pointmap.h"

using namespace std;

// Custom types
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// ICP params and result classes
// *****************************

class ICP_params {
 public:
  // Elements
  // ********

  // number of points sampled at each step
  size_t n_samples;

  // Pairing distance threshold
  float max_pairing_dist;
  float max_planar_dist;

  // Convergence thresholds
  size_t max_iter;
  size_t avg_steps;
  float rotDiffThresh;    // Stop threshold on the Variation of R
  float transDiffThresh;  // Stop threshold on the Variation of T

  // For motion distortion, angle phi of the last transform
  float init_phi;
  bool motion_distortion;

  // Initial transformation
  Eigen::Matrix4d init_transform;

  // Methods
  // *******

  // Constructor
  ICP_params() {
    n_samples = 1000;
    max_pairing_dist = 5.0;
    max_planar_dist = 0.3;
    max_iter = 50;
    avg_steps = 3;
    rotDiffThresh = 0.1 * M_PI / 180.0;
    transDiffThresh = 0.01;
    init_phi = 0.0;
    motion_distortion = false;
    init_transform = Eigen::Matrix4d::Identity(4, 4);
  }
};

std::ostream& operator<<(std::ostream& os, const ICP_params& s);

class ICP_results {
 public:
  // Elements
  // ********

  // Final transformation
  Eigen::Matrix4d transform;
  Eigen::MatrixXd all_transforms;

  // Final RMS error
  vector<float> all_rms;
  vector<float> all_plane_rms;

  // Methods
  // *******

  // Constructor
  ICP_results() {
    transform = Eigen::Matrix4d::Identity(4, 4);
    all_transforms = Eigen::MatrixXd::Zero(4, 4);
    all_rms = vector<float>();
    all_plane_rms = vector<float>();
  }
};

class BundleIcpResults {
 public:
  // Elements
  // ********

  // Final transformations
  vector<Eigen::Matrix4d> transforms;
  Eigen::MatrixXd all_transforms;

  // Final RMS error
  vector<vector<float>> all_rms;

  // Methods
  // *******

  // Constructor
  BundleIcpResults(int bundle_size) {
    transforms =
        vector<Eigen::Matrix4d>(bundle_size, Eigen::Matrix4d::Identity(4, 4));
    all_rms = vector<vector<float>>(bundle_size);
    all_transforms = Eigen::MatrixXd::Zero(4, 4 * bundle_size);
  }
};

// Function declaration
// ********************

Eigen::Matrix4d pose_interp(float t, Eigen::Matrix4d const& H1,
                            Eigen::Matrix4d const& H2, int verbose);

void SolvePoint2PlaneLinearSystem(const Matrix6d& A, const Vector6d& b,
                                  Vector6d& x);

void PointToPlaneErrorMinimizer(vector<PointXYZ>& targets,
                                vector<PointXYZ>& references,
                                vector<PointXYZ>& refNormals,
                                vector<float>& weights,
                                vector<pair<size_t, size_t>>& sample_inds,
                                Eigen::Matrix4d& mOut);

void PointToMapICPDebug(vector<PointXYZ>& tgt_pts, vector<float>& tgt_w,
                        vector<PointXYZ>& map_points,
                        vector<PointXYZ>& map_normals,
                        vector<float>& map_scores, ICP_params& params,
                        ICP_results& results);

void PointToMapICP(vector<PointXYZ>& tgt_pts, vector<float>& tgt_w,
                   PointMap& map, ICP_params& params, ICP_results& results);

void BundleICP(vector<PointXYZ>& points, vector<PointXYZ>& normals,
               vector<float>& weights, vector<int>& lengths, ICP_params& params,
               BundleIcpResults& results);
