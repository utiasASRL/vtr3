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
  
  // doppler calib parameters
  config->azimuth_res = node->declare_parameter<double>(param_prefix + ".azimuth_res", config->azimuth_res);
  config->azimuth_start = node->declare_parameter<double>(param_prefix + ".azimuth_start", config->azimuth_start);
  config->azimuth_end = node->declare_parameter<double>(param_prefix + ".azimuth_end", config->azimuth_end);
  config->num_rows = node->declare_parameter<int>(param_prefix + ".num_rows", config->num_rows);
  config->num_cols = node->declare_parameter<int>(param_prefix + ".num_cols", config->num_cols);
  config->min_dist = node->declare_parameter<double>(param_prefix + ".min_dist", config->min_dist);
  config->max_dist = node->declare_parameter<double>(param_prefix + ".max_dist", config->num_cols);
  //
  config->root_path = node->declare_parameter<std::string>(param_prefix + ".root_path", config->root_path);
  config->model_name = node->declare_parameter<std::string>(param_prefix + ".model_name", config->model_name);
  config->downsample_steps = node->declare_parameter<int>(param_prefix + ".downsample_steps", config->downsample_steps);
  config->bias_input_feat =node->declare_parameter<std::vector<std::string>>(param_prefix + ".bias_input_feat", config->bias_input_feat);
  config->var_input_feat = node->declare_parameter<std::vector<std::string>>(param_prefix + ".var_input_feat", config->var_input_feat);
  config->bias_polyorder = node->declare_parameter<int>(param_prefix + ".bias_polyorder", config->bias_polyorder);
  config->var_polyorder = node->declare_parameter<int>(param_prefix + ".var_polyorder", config->var_polyorder);
  //
  config->calc_median = node->declare_parameter<bool>(param_prefix + ".calc_median", config->calc_median);
  config->calc_pseudovar = node->declare_parameter<bool>(param_prefix + ".calc_pseudovar", config->calc_pseudovar);
  //
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

float atan2_approx(float y, float x) {
  float pi = M_PI;
  float pi_2 = M_PI_2;

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

void PreprocessingDopplerModule::initImgWeight(bool set_dims, const Config::ConstPtr &config, const std::string& dim_txt, const std::string& binary, std::vector<std::vector<ImgWeight>>& weights) {
  // read csv that specifies dimensions
  std::ifstream csv(dim_txt);
  Eigen::MatrixXi dims = readCSVtoEigenXd(csv).cast<int>();  // 0(# sensors) x 1(# rows) x 2(# cols) x 3(# faces) x 4(weight dim)

  // reassigns values for rows and cols based on weights data
  if (set_dims) {
    config_->num_rows = dims(1);
    config_->num_cols = dims(2);
  }

  Eigen::VectorXd dummy_vec(dims(4)); // dummy vector with appropriate size
  ImgWeight bias_weight(dims(1), std::vector<Eigen::VectorXd>(dims(2), dummy_vec)); // (# rows) x (# cols) x (weight dim)

  // initialize with approriate (# sensors) x (# faces)
  weights = std::vector<std::vector<ImgWeight>>(dims(0), std::vector<ImgWeight>(dims(3), bias_weight));

  // read binary
  std::ifstream ifs(binary, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4;
  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float { return *((float *)(byteArray + index)); };

  for (size_t sensor = 0; sensor < dims(0); ++sensor) {
    for (size_t row = 0; row < dims(1); ++row) {
      for (size_t col = 0; col < dims(2); ++col) {
        for (size_t face = 0; face < dims(3); ++face) {
          for (size_t d = 0; d < dims(4); ++d) {  
            int offset = d + dims(4)*face + dims(4)*dims(3)*col 
              + dims(4)*dims(3)*dims(2)*row + dims(4)*dims(3)*dims(2)*dims(1)*sensor;
            weights[sensor][face][row][col](d) = getFloatFromByteArray(buffer.data(), offset * float_offset);
          } // d
        } // face
      } // col
    } // row
  } // sensor
}

void PreprocessingDopplerModule::buildFeatVec(Eigen::VectorXd& feat, const PointWithInfo& point, 
    const std::vector<std::string>& feat_string, double dop_median, double dop_pseudovar) const {

  auto scaleValue = [](double value, double lower, double upper) -> double { return 2.0*(value - lower)/(upper - lower) - 1.0; }; 

  for (size_t i = 0; i < feat_string.size(); ++i) {
    double val = 0;
    if (feat_string[i] == "range")
      val = scaleValue(point.rho, config_->min_dist, config_->max_dist);  // note: range is calculated in preprocess function of DopplerFilter
    else if (feat_string[i] == "intensity")
      val = scaleValue(point.intensity, -70.0, 0.0);
    else if (feat_string[i] == "medianrv") {
      val = scaleValue(dop_median, -30.0, 1.0); // for forward-facing sensor
    }
    else if (feat_string[i] == "rv_var5") // TODO: handle variable
      val = scaleValue(std::min(1.0 / sqrt(dop_pseudovar), 200.0), 0.0, 200.0);
    else if (feat_string[i] == "rv_stddev5") // TODO: handle variable
      val = scaleValue(sqrt(dop_pseudovar), 0.0, 1.0);
    else
      throw std::runtime_error("[DopplerImageCalib::buildFeatVec] Unknown feature!");
    feat(i) = val;  // set value
  }
}

double PreprocessingDopplerModule::computeModel(const Eigen::VectorXd& feat, const Eigen::VectorXd& weights, int polyorder) const {
  if (polyorder * feat.size() + 1 != weights.size()) {
    CLOG(WARNING, "lidar.preprocessing_doppler") << "[DopplerImageCalib::computeModel] Incompatible feature and weight dimensions!" 
                 << polyorder << ", " << feat.size() << ", " << weights.size() << std::endl;
    throw std::runtime_error("[DopplerImageCalib::computeModel] Incompatible feature and weight dimensions!");
  }
  
  double output = 0;
  Eigen::VectorXd featpow = feat;
  // Eigen::VectorXd featpow = Eigen::VectorXd::Ones(feat.size()); // TODO: bug in training code that starts with feat^0
  for (int i = 0; i < polyorder; ++i) {
    output += featpow.dot(weights.segment(i * feat.size(), feat.size()));
    featpow.array() *= feat.array();
  }
  output += weights(weights.size() - 1);  // bias term
  return output;
}

bool PreprocessingDopplerModule::computePseudovar(double& pseudovar, const std::vector<const PointWithInfo*>& img_row, int c, int hwidth, double tol) const {
  pseudovar = 1.0;  // default value
  double vbar = img_row[c]->radial_velocity;  // use value at (r, c) as the "mean"
  int count = 0;  // count for calculating variance
  double sum = 0; // sum for calculating variance

  // loop from -hwidth to +hwidth
  for (int ci = c - hwidth; ci <= c + hwidth; ++ci) {
    if (ci == c || ci < 0 || ci >= config_->num_cols || img_row[ci] == nullptr)
      continue; // skip if ci is at c, or no measurement at (r, ci), or beyond limits

    double diff = img_row[ci]->radial_velocity - vbar;
    if (fabs(diff) > tol)
      continue; // skip if value difference is beyond tolerance

    // okay to add
    sum += (diff * diff);
    ++count;
  } // end for ci

  if (count > 3 && sum != 0) {
    pseudovar = sum/count; // set pseudovariance
    return true;
  }
  else
    return false;     // skip this measurement since there are no neighbours
}

PreprocessingDopplerModule::PreprocessingDopplerModule(const Config::ConstPtr &config, const std::shared_ptr<tactic::ModuleFactory> &module_factory, const std::string &name) : tactic::BaseModule(module_factory, name), config_(config) {
  // init weights
  std::string bias_shape = config_->root_path + "/" + config_->model_name + "/bias_shape.txt"; 
  std::string binary = config_->root_path + "/" + config_->model_name + "/bias.bin";
  initImgWeight(true, config, bias_shape, binary, bias_weights_);
  
  std::string var_shape = config_->root_path + "/" + config_->model_name + "/var_shape.txt"; 
  std::string var = config_->root_path + "/" + config_->model_name + "/var.bin";
  initImgWeight(false, config, var_shape, var, var_weights_);

  // check if we need to calculate median Doppler velocity
  for (const auto& feat: config_->bias_input_feat)
    if (feat == "medianrv")
      config_->calc_median = true;
  for (const auto& feat: config_->var_input_feat) {
    if (feat == "medianrv")
      config_->calc_median = true;
    if (feat == "rv_var5")  // TODO: handle variable parameter in string
      config_->calc_pseudovar = true;
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

  // 2D vector of pointers to points
  using PointImg = std::vector<std::vector<const PointWithInfo*>>;

  // initialize empty grid (2D img filled with null pointers)
  PointImg empty_img(config_->num_rows, std::vector<const PointWithInfo*>(config_->num_cols, nullptr));

  // create an image for the one aeva sensor
  std::vector<PointImg> imgs(1, empty_img);
  int pt_count = 0;  // keeps track of total # points to reserve later
  std::vector<double> dop_vels; // doppler median

  // iterate over each point
  for (size_t i = 0; i < filtered_point_cloud.size(); i++) {

    if (filtered_point_cloud[i].rho < config_->min_dist || filtered_point_cloud[i].rho > config_->max_dist)
      continue;  // skip if not within range

    // polynomial approx. of atan2
    const double azimuth = atan2_approx(filtered_point_cloud[i].y, filtered_point_cloud[i].x);  // approximation slightly faster than atan2 call

    // skip if not within azimuth bounds (horizontal fov) 
    if (azimuth <= config_->azimuth_start || azimuth >= config_->azimuth_end)
      continue;

    // determine column
    const short col = (config_->num_cols - 1) - int((azimuth - config_->azimuth_start)/config_->azimuth_res);

    // bounds check for line_id and col
    const int line_id = filtered_point_cloud[i].line_id;
    if (line_id < 0 || line_id >= config_->num_rows)
      continue;
    if (col < 0 || col >= imgs[0][line_id].size())
      continue;

    // picking the closest in elevation
    if (imgs[0][line_id][col] == nullptr) {
      // keep first measurement in bin
      imgs[0][line_id][col] = &filtered_point_cloud[i];
      ++pt_count;

      // stack velocities for median calculation
      if (config_->calc_median)
        dop_vels.push_back(filtered_point_cloud[i].radial_velocity);
    }
  }

  // median calculation
  double dop_median;
  if (config_->calc_median) {
    int n = dop_vels.size()/2;
    auto nitr = dop_vels.begin() + n;
    std::nth_element(dop_vels.begin(), nitr, dop_vels.end());
    dop_median = *nitr;
  }

  // output
  pcl::PointCloud<PointWithInfo> out_frame;
  out_frame.reserve(pt_count);
  Eigen::VectorXd bias_feat(config_->bias_input_feat.size());
  Eigen::VectorXd var_feat(config_->var_input_feat.size());
  int dscount = 0;
  for (size_t r = 0; r < config_->num_rows; ++r) {
    for (size_t c = 0; c < config_->num_cols; ++c) {
      if (imgs[0][r][c] != nullptr) {

        // step downsample after image projection
        ++dscount;
        if (dscount % config_->downsample_steps != 0)
          continue;

        // pseudo-variance
        double pseudovar = 1.0;
        if (config_->calc_pseudovar) {
          bool varflag = computePseudovar(pseudovar, imgs[0][r], c, pseudo_var_hwidth_, 9999);
          if (!varflag)
            continue;
        }

        // pushback if we have data in this elevation-azimuth bin
        out_frame.push_back(*imgs[0][r][c]);
        
        // build features
        buildFeatVec(bias_feat, out_frame.back(), config_->bias_input_feat, dop_median, pseudovar);
        buildFeatVec(var_feat, out_frame.back(), config_->var_input_feat, dop_median, pseudovar);

        // apply linear regression model
        int faceid = out_frame.back().face_id;
        out_frame.back().radial_velocity -= computeModel(bias_feat, bias_weights_[0][faceid][r][c], config_->bias_polyorder);
        out_frame.back().ivariance = exp(computeModel(var_feat, var_weights_[0][faceid][0][0], config_->var_polyorder)); // TODO: when var is also a grid/image
      }
    }
  }

  auto filtered_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(out_frame);

  // print size after regression step
  CLOG(DEBUG, "lidar.preprocessing_doppler")
      << "final point size: " << filtered_cloud->size();

  if (config_->visualize) {
    PointCloudMsg pc2_msg;
    pcl::toROSMsg(*filtered_cloud, pc2_msg);
    pc2_msg.header.frame_id = "lidar";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(pc2_msg);
  }

  /// Output
  qdata.doppler_preprocessed_point_cloud = filtered_cloud;

}

}  // namespace lidar
}  // namespace vtr