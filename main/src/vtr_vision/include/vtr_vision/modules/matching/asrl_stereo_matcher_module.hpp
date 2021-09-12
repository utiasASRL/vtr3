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
 * \file asrl_stereo_matcher_module.hpp
 * \brief ASRLStereoMatcherModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>
#include <vtr_vision/visualize.hpp>

namespace vtr {
namespace vision {

/**
 * \brief Reject outliers and estimate a preliminary transform
 * \details
 * requires:
 *   qdata.[rig_calibrations, rig_features, candidate_landmarks,
 *          T_sensor_vehicle, map_landmarks, T_r_m_prior, T_sensor_vehicle_map]
 * outputs:
 *   qdata.[raw_matches]
 */
class ASRLStereoMatcherModule : public tactic::BaseModule {
 public:
  static constexpr auto static_name = "asrl_stereo_matcher";

  // use a method to predict where pixels have moved
  enum class PredictionMethod { none = 0, se3 = 1 };

  struct Config {
    /**
     * \brief Check the laplacian bits are the same when matching two keypoints
     */
    bool check_laplacian_bit = true;

    /** \brief Check the octaves are the same when matching two keypoints */
    bool check_octave = true;

    /** \brief Check the responses are the same when matching two keypoints */
    bool check_response = false;

    /**
     * \brief The minimum ratio between the two responses.
     * \details
     * A value of 1.0 means they must be exactly the same
     * A value of 0.0 means they can be infinitely different
     * A value of about 0.1 is a good place to start
     */
    double min_response_ratio = 0.2;

    //
    //    /** \brief The maximum distance threshold between a candidate
    //    ///        match, in pixels. This will move with the prediction
    //    ///        method's prediction of pixel location
    //     */
    //
    //    int window_size;

    /// The maximum distance threshold between a candidate match [pixels].
    int matching_pixel_thresh = 100;

    /// The maximum distance threshold between a candidate match [pixels],
    /// when we are very confident.
    int tight_matching_pixel_thresh = 50;

    /// The standard deviation threshold for pose uncertainty to qualify for
    /// tight matching
    double tight_matching_x_sigma = 0.4;
    double tight_matching_y_sigma = 0.4;
    double tight_matching_theta_sigma = 0.4;
#if false
    /**
     * \brief Scales the search window to reduce low precision feature search
     * windows, like so:
     * search_window_in_pixels =
     *   window_size*(sqrt(1.0/precision)^window_pow_scale
     */
    double window_pow_scale;
#endif
    /**
     * \brief Whether or not to use the precision associated with the keypoint
     * to inflate the window size
     */
    bool use_pixel_variance = true;

    /** \brief Use an estimate of motion to predict where pixels have moved */
    PredictionMethod prediction_method = PredictionMethod::se3;

    /** \brief The maximum point depth considered for matches. */
    double max_point_depth = 1000;

    /** \brief The maximum Euclidean distance threshold for a match. */
    double descriptor_thresh = 0.1;

    /** \brief The number of openmp threads to use when matching */
    int parallel_threads = 8;

    /** \brief Whether or not to visualise the matches */
    bool visualize_feature_matches = false;

    /** \brief How many features are found to constitute a success */
    int min_matches = 0;
  };

  /** \brief Constructor */
  ASRLStereoMatcherModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {
    use_tight_pixel_thresh_ = false;
    force_loose_pixel_thresh_ = false;
  }

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

  /** \brief Perform the feature matching */
  unsigned matchFeatures(CameraQueryCache &qdata,
                         const tactic::Graph::ConstPtr &graph);

 private:
  /**
   * \brief This module matches a query stereo frame to a map stereo frame and
   * fills in the inlier between them.
   */
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  /** \brief Visualization implementation */
  void visualizeImpl(tactic::QueryCache &qdata,
                     const tactic::Graph::ConstPtr &graph) override;

  /**
   * \brief Check the keypoint data to see if it's worth doing a descriptor
   * match
   */
  bool checkConditions(const Keypoint &kp_map, const FeatureInfo &lm_info_map,
                       const Keypoint &kp_query, const FeatureInfo &lm_info_qry,
                       const cv::Point &qry_pt, const cv::Point &map_pt);

  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;

  bool use_tight_pixel_thresh_;
  bool force_loose_pixel_thresh_;
};

}  // namespace vision
}  // namespace vtr
