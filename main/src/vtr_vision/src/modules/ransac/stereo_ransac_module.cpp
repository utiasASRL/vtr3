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
 * \file stereo_ransac_module.cpp
 * \brief StereoRansacModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/modules/ransac/stereo_ransac_module.hpp>
#include "vtr_logging/logging.hpp"

namespace vtr {
namespace vision {

using namespace tactic;

auto StereoRansacModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) 
    -> ConstPtr {
  auto config = RansacModule::Config::fromROS(node, param_prefix);

  auto stereo_config = std::make_shared<Config>();
  auto casted_config =
      std::static_pointer_cast<RansacModule::Config>(stereo_config);
  *casted_config = *config;  // copy over base config

  // clang-format off
  stereo_config->mask_depth = node->declare_parameter<double>(param_prefix + ".mask_depth", stereo_config->mask_depth);
  stereo_config->mask_depth_inlier_count = node->declare_parameter<int>(param_prefix + ".mask_depth_inlier_count", stereo_config->mask_depth_inlier_count);
  stereo_config->use_covariance = node->declare_parameter<bool>(param_prefix + ".use_covariance", stereo_config->use_covariance);
  return stereo_config;

  // clang-format on
}

std::shared_ptr<vision::SensorModelBase<Eigen::Matrix4d>>
StereoRansacModule::generateRANSACModel(CameraQueryCache &qdata) {

  // Get the data from the cache.
  const auto &calibrations = *qdata.rig_calibrations;

  // set up the model.
  auto stereo_model = std::make_shared<vision::StereoTransformModel>();

  // set up the measurement variance
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inv_r_matrix;

  // Set up the map and query points based on whether
  // we are configured to use migrated points or not.
  if (stereo_config_->use_migrated_points) {
    auto &query_landmarks = *qdata.map_landmarks;
    auto map_points = &(*qdata.migrated_points_3d);

    // clear any old data
    query_points_.reset(new EigenMatrix3Dynamic);

    // grab the points from the landmarks and put them in a contiguous matrix
    addPointsFromLandmarks(*query_points_.get(), query_landmarks[0].landmarks,
                           query_channel_offsets_);

    // add the map and query points to the stereo model
    stereo_model->setPoints(map_points, query_points_.get());

    // keep a record of the channel offsets
    for (uint32_t channel_idx = 0; channel_idx < query_channel_offsets_.size();
         ++channel_idx) {
      map_channel_offsets_[channel_idx] =
          std::pair<int, int>(0, map_points->cols());
    }
    // TODO: (old) Add migrated covariance in, need to set up noise evaluator
    // based on prior in landmark migration module.

  } else {
    auto &query_landmarks = *qdata.candidate_landmarks;
    auto &map_landmarks = *qdata.map_landmarks;

    // clear any old data
    map_points_.reset(new EigenMatrix3Dynamic);
    query_points_.reset(new EigenMatrix3Dynamic);

    // grab the points from the landmarks and put them in a contiguous matrix
    addPointsFromLandmarks(*map_points_.get(), map_landmarks[0].landmarks,
                           map_channel_offsets_);
    addPointsFromLandmarks(*query_points_.get(), query_landmarks[0],
                           query_channel_offsets_);

    // set the covariances for each observation
    if (stereo_config_->use_covariance) {
      setCovarianceFromObservations(inv_r_matrix, map_landmarks[0].observations,
                                    map_channel_offsets_);
    }

    // add the measurement variances to the stereo model
    stereo_model->setMeasurementVariance(inv_r_matrix);

    // add the map and query points to the stereo model
    stereo_model->setPoints(map_points_.get(), query_points_.get());
  }

  // TODO: (old) Use covariance in RANSAC for migrated points.
  // TODO: (old) Multi-Channel.

  // Set up the calibration.
  auto extrinsic = calibrations.front().extrinsics[1];
  auto baseline = -extrinsic.matrix()(0, 3);
  auto intrinsic = calibrations.front().intrinsics[0];
  stereo_model->setCalibration(intrinsic, baseline);

  return stereo_model;
}

void StereoRansacModule::setCovarianceFromObservations(
    vision::MeasVarList &inv_r_matrix,
    const vision::RigObservations &observations, OffsetMap &) {
  // figure out how many landmarks there are across all of the channels.
  int num_landmarks = 0;
  for (auto &channel : observations.channels) {
    if (!channel.cameras.empty())
      num_landmarks += channel.cameras[0].covariances.size();
  }

  // set up the points for the ransac problem.
  int lm_idx = 0;
  inv_r_matrix.conservativeResize(2, 2 * (num_landmarks + inv_r_matrix.cols()));

  // Go through every channel.
  for (const auto &channel : observations.channels) {
    // keep track of the offset into this channel.
    if (!channel.cameras.empty()) {
      for (unsigned idx = 0; idx < channel.cameras[0].precisions.size();
           idx++) {
        // grab the inverse covariances
        inv_r_matrix.block(0, 2 * (lm_idx + idx), 2, 2) =
            channel.cameras[0].covariances[idx].inverse();
      }
      lm_idx += channel.cameras[0].precisions.size();
    }
  }
}

void StereoRansacModule::addPointsFromLandmarks(
    Eigen::Matrix<double, 3, Eigen::Dynamic> &ransac_points,
    const vision::RigLandmarks &landmarks, OffsetMap &channel_offsets) {
  // figure out how many landmarks there are across all of the channels.
  int num_landmarks = 0;
  for (auto &channel : landmarks.channels) {
    num_landmarks += channel.points.cols();
  }

  // set up the points for the ransac problem.
  int lm_idx = 0;
  ransac_points.conservativeResize(3, num_landmarks + ransac_points.cols());

  // Go through every channel.
  for (uint32_t channel_idx = 0; channel_idx < landmarks.channels.size();
       channel_idx++) {
    auto &channel = landmarks.channels[channel_idx];
    // keep track of the offset into this channel.
    channel_offsets[channel_idx] =
        std::pair<int, int>(lm_idx, lm_idx + channel.points.cols());
    if (channel.points.cols() > 0) {
      // set the point.
      ransac_points.block(0, lm_idx, 3, channel.points.cols()) = channel.points;
      lm_idx += channel.points.cols();
    }
  }
}

std::shared_ptr<vision::BasicSampler> StereoRansacModule::generateRANSACSampler(
    CameraQueryCache &qdata) {
  auto &query_landmarks = *qdata.candidate_landmarks;
  CLOG(INFO, "stereo.ransac") << "accessing query landmarks"; 
  std::vector<bool> mask;
  for (auto &rig : query_landmarks) {
    for (auto &channel : rig.channels) {
      CLOG(INFO, "stereo.ransac") << "accessing rig channels" << channel.points.cols(); 
      for (uint32_t lm_idx = 0; lm_idx < channel.points.cols(); ++lm_idx) {
        CLOG(DEBUG, "stereo.ransac") << "z depth val:" << channel.points(2, lm_idx);
        CLOG(DEBUG, "stereo.ransac") << "mask_depth:" << stereo_config_->mask_depth;
        mask.push_back(channel.points(2, lm_idx) < stereo_config_->mask_depth);
      }
    }
  }

  auto verifier = std::make_shared<vision::VerifySampleSubsetMask>(
      stereo_config_->mask_depth_inlier_count, mask);  // Need 1 close feature
  
  return std::make_shared<vision::BasicSampler>(verifier);
}

}  // namespace vision
}  // namespace vtr