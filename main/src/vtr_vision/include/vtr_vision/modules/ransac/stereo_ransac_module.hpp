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
 * \file stereo_ransac_module.hpp
 * \brief StereoRansacModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <random>

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_vision/modules/ransac/ransac_module.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_vision/outliers/sampler/basic_sampler.hpp>
#include <vtr_vision/outliers/sampler/verify_sample_subset_mask.hpp>
#include <vtr_vision/sensors/stereo_transform_model.hpp>

using EigenMatrix3Dynamic = Eigen::Matrix<double, 3, Eigen::Dynamic>;

namespace vtr {
namespace vision {

/**
 * \brief Reject outliers and estimate a preliminary transform using Stereo
 * Data
 * \details
 * requires:
 *   qdata.[candidate_landmarks, rig_features, rig_calibrations,
 *          T_sensor_vehicle, live_id, map_landmarks, T_q_m_prior,
 *          T_sensor_vehicle_map, raw_matches]
 * outputs:
 *   qdata.[T_q_m, ransac_matches, success, **steam_failure]
 *
 * The preliminary transform is stored in T_q_m, and inliers stored in
 * ransac_matches.
 * \todo (yuchen) why is steam_failure relevant in this module? likely for
 * testing only, should be able to remove it
 */
class StereoRansacModule : public RansacModule {
 public:
  /**
   * \brief Static module identifier.
   * \todo change this to static_name
   */
  static constexpr auto static_name = "stereo_ransac";

  /** \brief Collection of config parameters */
  struct Config : RansacModule::Config {
    double mask_depth = 200.0;
    int mask_depth_inlier_count = 0;
    // Whether or not to use landmark covariance for the error model
    bool use_covariance = false;
  };

  /** \brief TODO Construct with settings... */
  StereoRansacModule(std::string name = static_name)
      : RansacModule{name},
        doom_twister(vo_doom_generator()),
        doom_distribution(0, 100) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 protected:
  /**
   * \brief Generates a model for the RANSAC method.
   * \param[in] qdata query data cache
   * \return A pointer to the RANSAC model.
   */
  std::shared_ptr<SensorModelBase<Eigen::Matrix4d>> generateRANSACModel(
      CameraQueryCache &qdata) override;

  /**
   * \brief Generates a sampler for the RANSAC method.
   * \param[in] qdata query data cache
   * \return A pointer to the RANSAC model.
   */
  std::shared_ptr<BasicSampler> generateRANSACSampler(
      CameraQueryCache &qdata) override;

 private:
  /**
   * \brief Adds points to the ransac problem given a set of landmarks.
   * \param[in,out] ransac_points the points to be added to
   * \param[in] landmarks the landmarks.
   * \param[in] channel_offsets a set of offsets corresponding to each channel.
   */
  void addPointsFromLandmarks(
      Eigen::Matrix<double, 3, Eigen::Dynamic> &ransac_points,
      const RigLandmarks &landmarks, OffsetMap &channel_offsets);

  /**
   * \brief Adds covariances to the ransac problem given a set of landmarks.
   * \param[in,out] inv_r_matrix the covariant to be added to
   * \param[in] landmarks the landmarks.
   */
  void setCovarianceFromObservations(MeasVarList &inv_r_matrix,
                                     const RigObservations &landmarks,
                                     OffsetMap &);

  /** \brief The flattened query points. */
  std::shared_ptr<EigenMatrix3Dynamic> query_points_;

  /** \brief The flattened map points. */
  std::shared_ptr<EigenMatrix3Dynamic> map_points_;

  std::shared_ptr<Config> stereo_config_;

  std::random_device vo_doom_generator;  // non-deterministic generator
  std::mt19937 doom_twister;  // (rd());  // to seed mersenne twister.
                              // replace the call to rd() with a
                              // constant value to get repeatable
                              // results.
  std::uniform_real_distribution<double> doom_distribution;
};

}  // namespace vision
}  // namespace vtr
