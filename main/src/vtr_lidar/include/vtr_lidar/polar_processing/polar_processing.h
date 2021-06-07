#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <set>

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Dense>

#include "vtr_lidar/cloud/cloud.h"
#include "vtr_lidar/nanoflann/nanoflann.hpp"
#include "vtr_lidar/pointmap/pointmap.h"

using namespace std;

// KDTree type definition
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>
    PointXYZ_KDTree;

void cart2pol_(vector<PointXYZ>& xyz);
PointXYZ cart2pol(const PointXYZ& p);

void pca_features(vector<PointXYZ>& points, vector<float>& eigenvalues,
                  vector<PointXYZ>& eigenvectors);

void detect_outliers(vector<PointXYZ>& rtp, vector<float>& scores,
                     int lidar_n_lines, float lidar_angle_res, float minTheta,
                     int n_pass, float threshold);

float get_lidar_angle_res(vector<PointXYZ>& rtp, float& minTheta,
                          float& maxTheta, int lidar_n_lines);

void lidar_log_radius(vector<PointXYZ>& rtp, float polar_r, float r_scale);

void lidar_horizontal_scale(vector<PointXYZ>& rtp, float h_scale);

void extract_features_multi_thread(vector<PointXYZ>& points,
                                   vector<PointXYZ>& normals,
                                   vector<float>& planarity,
                                   vector<float>& linearity, int lidar_n_lines,
                                   float h_scale, float r_scale, int verbose);

void smart_normal_score(vector<PointXYZ>& points, vector<PointXYZ>& polar_pts,
                        vector<PointXYZ>& normals, vector<float>& scores);

void smart_icp_score(vector<PointXYZ>& polar_pts, vector<float>& scores);

void compare_map_to_frame(
    vector<PointXYZ>& frame_points, vector<PointXYZ>& map_points,
    vector<PointXYZ>& map_normals, unordered_map<VoxKey, size_t>& map_samples,
    Eigen::Matrix3d R_d, Eigen::Vector3d T_d, float theta_dl, float phi_dl,
    float map_dl, vector<float>& movable_probs, vector<int>& movable_counts);

void extract_lidar_frame_normals(vector<PointXYZ>& points,
                                 vector<PointXYZ>& polar_pts,
                                 vector<PointXYZ>& queries,
                                 vector<PointXYZ>& polar_queries,
                                 vector<PointXYZ>& normals,
                                 vector<float>& norm_scores, float polar_r,
                                 int parallel_threads = 1);
