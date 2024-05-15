// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file preprocessing_module.cpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/preprocessing_module_doppler.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/features/normal.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

#include <fstream>
#include <iostream>

namespace vtr {
namespace lidar {


using namespace tactic;

auto PreprocessingDopplerModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);

  config->crop_range = node->declare_parameter<float>(param_prefix + ".crop_range", config->crop_range);
  config->vertical_angle_res = node->declare_parameter<float>(param_prefix + ".vertical_angle_res", config->vertical_angle_res);
  config->polar_r_scale = node->declare_parameter<float>(param_prefix + ".polar_r_scale", config->polar_r_scale);
  config->r_scale = node->declare_parameter<float>(param_prefix + ".r_scale", config->r_scale);
  config->h_scale = node->declare_parameter<float>(param_prefix + ".h_scale", config->h_scale);
  config->frame_voxel_size = node->declare_parameter<float>(param_prefix + ".frame_voxel_size", config->frame_voxel_size);
  config->nn_voxel_size = node->declare_parameter<float>(param_prefix + ".nn_voxel_size", config->nn_voxel_size);


  config->filter_by_normal_score = node->declare_parameter<bool>(param_prefix + ".filter_normal", config->filter_by_normal_score);
  config->num_sample1 = node->declare_parameter<int>(param_prefix + ".num_sample1", config->num_sample1);
  config->min_norm_score1 = node->declare_parameter<float>(param_prefix + ".min_norm_score1", config->min_norm_score1);

  config->num_sample2 = node->declare_parameter<int>(param_prefix + ".num_sample2", config->num_sample2);
  config->min_norm_score2 = node->declare_parameter<float>(param_prefix + ".min_norm_score2", config->min_norm_score2);
  config->min_normal_estimate_dist = node->declare_parameter<float>(param_prefix + ".min_normal_estimate_dist", config->min_normal_estimate_dist);
  config->max_normal_estimate_angle = node->declare_parameter<float>(param_prefix + ".max_normal_estimate_angle", config->max_normal_estimate_angle);

  config->cluster_num_sample = node->declare_parameter<int>(param_prefix + ".cluster_num_sample", config->cluster_num_sample);

  // doppler calib parameters
  // config->azimuth_res = node->declare_parameter<double>(param_prefix + ".azimuth_res", config->azimuth_res);
  config->azimuth_start = node->declare_parameter<double>(param_prefix + ".azimuth_start", config->azimuth_start);
  config->azimuth_end = node->declare_parameter<double>(param_prefix + ".azimuth_end", config->azimuth_end);
  config->num_rows = node->declare_parameter<int>(param_prefix + ".num_rows", config->num_rows);
  config->num_cols = node->declare_parameter<int>(param_prefix + ".num_cols", config->num_cols);
  config->max_dist = node->declare_parameter<int>(param_prefix + ".max_dist", config->max_dist);
  //
  config->active_sensors = node->declare_parameter<std::vector<bool>>(param_prefix + ".active_sensors", config->active_sensors);
  config->root_path = node->declare_parameter<std::string>(param_prefix + ".root_path", config->root_path);


  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

float atan2_approx(float y, float x, float pi, float pi_2) {
  bool swap = fabs(x) < fabs(y);
  float atanin = (swap ? x : y) / (swap ? y : x);
  float a1 = 0.99997726;
  float a3 = -0.33262347;
  float a5 = 0.19354346;
  float a7 = -0.11643287;
  float a9 = 0.05265332;
  float a11 = -0.01172120;
  float atanin2 = atanin*atanin;
  float atanout = atanin * (a1 + atanin2 * (a3 + atanin2 * (a5 + atanin2 * (a7 + atanin2 * (a9 + atanin2 * a11)))));
  atanout = swap ? (atanin >= 0.0 ? pi_2 : -pi_2) - atanout : atanout;
  if (x < 0.0) {
    atanout = (y >= 0.0 ? pi : -pi) + atanout;
  }  
  return atanout;
}

Eigen::MatrixXd readCSVtoEigenXd(std::ifstream &csv) {
  std::string line;
  std::string cell;
  std::vector<std::vector<double>> mat_vec;
  while (std::getline(csv, line)) {
    std::stringstream lineStream(line);
    std::vector<double> row_vec;
    while (std::getline(lineStream, cell, ',')) {
      row_vec.push_back(std::stof(cell));
    }
    mat_vec.push_back(row_vec);
  }
  Eigen::MatrixXd output = Eigen::MatrixXd(mat_vec.size(), mat_vec[0].size());
  for (int i = 0; i < (int)mat_vec.size(); ++i) output.row(i) = Eigen::VectorXd::Map(&mat_vec[i][0], mat_vec[i].size());
  return output;
}

PreprocessingDopplerModule::PreprocessingDopplerModule(const Config::ConstPtr &config, const std::shared_ptr<tactic::ModuleFactory> &module_factory, const std::string &name) : tactic::BaseModule(module_factory, name), config_(config) {
  int num_sensors = config_->active_sensors.size();

  // read elevation settings
  elevation_order_.clear();
  elevation_order_by_beam_id_.clear();
  for (int i = 0; i < num_sensors; ++i) {
    if (!config_->active_sensors[i]) {
      continue;
    }

    std::string path = config_->root_path + "/mean_elevation_beam_order_" + std::to_string(i);
    std::ifstream csv(path);
    if (!csv) throw std::ios::failure("Error opening csv file");
    elevation_order_.push_back(readCSVtoEigenXd(csv));
    const auto& temp = elevation_order_.back();

    std::vector<Eigen::MatrixXd> sensor_elevation_order;
    for (int j = 0; j < 4; ++j) {   // 4 beams   
      Eigen::MatrixXd elevation_order_for_this_beam(temp.rows()/4, 2);  // first column is mean elevation, second column is row id
      int h = 0;
      for (int r = 0; r < temp.rows(); ++r) {
        // first column is mean elevation. Second column is beam id
        if (temp(r, 1) == j) {
          elevation_order_for_this_beam(h, 0) = temp(r, 0);
          elevation_order_for_this_beam(h, 1) = r;
          ++h;
        }
      } // end for r
      assert(h == temp.rows()/4);
      sensor_elevation_order.push_back(elevation_order_for_this_beam);
    } // end for j
    assert(sensor_elevation_order.size() == 4); // 4 beams
    elevation_order_by_beam_id_.push_back(sensor_elevation_order);
  } // end for i

  // TODO: handle different regression models. Load right model according to a parameter (currently hardcoded)
  weights_.clear();
  for (int i = 0; i < num_sensors; ++i) {
    if (!config_->active_sensors[i])
      continue;

    std::vector<Eigen::MatrixXd> temp; temp.clear();
    for (int r = 0; r < config_->num_rows; ++r) {
      std::string path = config_->root_path + "/sensor" + std::to_string(i) + "_s1/lr_weights_row_" + std::to_string(r);
      
      std::ifstream csv(path);
      if (!csv) throw std::ios::failure("Error opening csv file");
      Eigen::MatrixXd dummy = readCSVtoEigenXd(csv);
      temp.push_back(dummy);
    }
    weights_.push_back(temp);
  }
}

void PreprocessingDopplerModule::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("filtered_point_cloud", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Get input point cloud
  const auto point_cloud = qdata.raw_point_cloud.ptr();

  if (point_cloud->size() == 0) {
    std::string err{"Empty point cloud."};
    CLOG(ERROR, "lidar.preprocessing_doppler") << err;
    throw std::runtime_error{err};
  }

  CLOG(DEBUG, "lidar.preprocessing_doppler")
      << "raw point cloud size: " << point_cloud->size();

  const auto &filtered_point_cloud = *qdata.raw_point_cloud;

  float pi = M_PI;
  float pi_2 = M_PI_2;

  // 2D vector of pointers to points
  using PointWFlag = std::pair<bool,const PointWithInfo*>;
  int grid_count = 0;
  std::vector<std::vector<PointWFlag>> grid(config_->num_rows, std::vector<PointWFlag>(config_->num_cols, PointWFlag(false, NULL)));
  // iterate over each point
  for (unsigned i = 0; i < filtered_point_cloud.size(); i++) {
    // point.range = sqrt(point.pt[0]*point.pt[0] + point.pt[1]*point.pt[1] + point.pt[2]*point.pt[2]);
    if (filtered_point_cloud[i].rho < config_->min_dist || filtered_point_cloud[i].rho > config_->max_dist)
      continue;

    // const double range = sqrt(filtered_point_cloud[i].x * filtered_point_cloud[i].x + filtered_point_cloud[i].y * filtered_point_cloud[i].y + filtered_point_cloud[i].z * filtered_point_cloud[i].z);
    // const double azimuth = atan2_approx(filtered_point_cloud[i].y, filtered_point_cloud[i].x, pi, pi_2);
    const double xy = sqrt(filtered_point_cloud[i].x * filtered_point_cloud[i].x + filtered_point_cloud[i].y * filtered_point_cloud[i].y);
    // double elevation = atan2(point.pt[2], xy);
    const double elevation = atan2_approx(filtered_point_cloud[i].z, xy, pi, pi_2);

    // CLOG(WARNING, "lidar.prepro") << "azi 1 " << azimuth;
    // CLOG(WARNING, "lidar.prepro") << "azi 2 " << filtered_point_cloud[i].phi;
    // CLOG(WARNING, "lidar.prepro") << "ele 1 " << elevation;
    // CLOG(WARNING, "lidar.prepro") << "ele 2 " << filtered_point_cloud[i].theta;

    // skip if not within azimuth bounds (horizontal fov)
    if (filtered_point_cloud[i].phi <= config_->azimuth_start || filtered_point_cloud[i].phi >= config_->azimuth_end)
      continue;

    // determine column
    const short col = (config_->num_cols - 1) - int((filtered_point_cloud[i].phi - config_->azimuth_start)/config_->azimuth_res);
  
    // determine row by matching by beam_id (0, 1, 2, or 3) and closest elevation to precalculated values
    // note: elevation_order_by_beam_id_[sensorid][point.beam_id] first column is mean elevation, second column is row id
    const auto ele_diff = elevation_order_by_beam_id_[0][filtered_point_cloud[i].flex24].col(0).array() - elevation; // to do: shouldn't be hardcoded for one sensor
    double min_val = ele_diff(0)*ele_diff(0);
    int min_id = 0;
    for (int i = 1; i < ele_diff.rows(); ++i) {
      const auto val = ele_diff(i) * ele_diff(i);
      if (val < min_val) {
        min_val = val;
        min_id = i;
      }
    }
    
    // picking the closest in elevation
    const short row = elevation_order_by_beam_id_[0][filtered_point_cloud[i].flex24](min_id, 1); // to do: shouldn't be hardcoded for one sensor
    if (!grid[row][col].first) {
      // keep first measurement in bin
      grid[row][col] = PointWFlag(true, &filtered_point_cloud[i]);
      ++grid_count;
    }
  }

  // print size after regression step
  CLOG(DEBUG, "lidar.preprocessing_doppler") << "grid_count: " << grid_count;

  // output
  pcl::PointCloud<PointWithInfo> out_frame;
  out_frame.reserve(grid_count);
  for (int r = 0; r < config_->num_rows; ++r) {
    for (int c = 0; c < config_->num_cols; ++c) {
      if (grid[r][c].first) {
        // pushback if we have data in this elevation-azimuth bin
        out_frame.push_back(*grid[r][c].second);
        // int sensorid = (*grid[r][c].second).sensor_id; 
        int sensorid = 0; // to do: add support for multiple sensors

        // apply linear regression model
        out_frame.back().radial_velocity -= (weights_[sensorid][r](c, 0) + weights_[sensorid][r](c, 1)*out_frame.back().rho/250.0);
      }
    }
  }

  auto filtered_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(out_frame);

  // print size after regression step
  CLOG(DEBUG, "lidar.preprocessing_doppler")
      << "final subsampled point size: " << filtered_cloud->size();

  if (config_->visualize) {
    PointCloudMsg pc2_msg;
    pcl::toROSMsg(*filtered_cloud, pc2_msg);
    pc2_msg.header.frame_id = "lidar";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(pc2_msg);
  }

  /// Output
  qdata.preprocessed_point_cloud = filtered_cloud;

}

}  // namespace lidar
}  // namespace vtr