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
// WITHOUT WARRANTIES OR CONDITIONS OF AN0Y KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file mel_matcher_module.hpp
 * \brief MelMatcherModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/timing/stopwatch.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_vision/messages/bridge.hpp>

namespace vtr {
namespace vision {

/**
 * \brief A module that matches the current live view to a multi-experience
 * map.
 * \details
 * requires:
 *   qdata.[T_sensor_vehicle, rig_names, map_id, T_r_m_prior,
 *          T_sensor_vehicle_map, map_landmarks, localization_map,
 *          landmark_offset_map, projected_map_points, migrated_points_3d,
 *          migrated_validity]
 * outputs:
 *   qdata.[raw_matches]
 */
class MelMatcherModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "mel_matcher";
  PTR_TYPEDEFS(MelMatcherModule);


  /** \brief Configuration */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);

    /** \brief The target number of matches to early exit */
    int target_match_count = 200;

    /** \brief The minimum number of matches needed. */
    int min_match_count = 20;

    /** \brief The minimum length of feature tracks on map landmarks. */
    int min_track_length = 1;

    /** \brief The maximum depth of a map landmark. */
    double max_landmark_depth = 200.0;

    /**
     * \brief The maximum distance threshold between a candidate match
     * [pixels].
     */
    int matching_pixel_thresh = 200;

    /**
     * \brief  The maximum distance threshold between a candidate match
     * [pixels], when we are very confident.
     */
    int tight_matching_pixel_thresh = 50;

    /**
     * \brief The standard deviation threshold for pose uncertainty to qualify
     * for tight matching
     */
    double tight_matching_x_sigma = 0.1;
    double tight_matching_y_sigma = 0.1;
    double tight_matching_theta_sigma = 0.0;

    /**
     * \brief The minimum ratio between the two responses.
     * A value of 1.0 means they must be exactly the same
     * A value of 0.0 means they can be infinitely different
     * A value of about 0.1 is a good place to start
     */
    double min_response_ratio = 0.2;

    /**
     * \brief Time allowance for the matching threshold. If the computation
     * time of the algorithm exceeds this number, it will exit with the current
     * found matches.
     */
    double time_allowance = 2000.0;

    /** \brief Threshold on descriptor distance. */
    double descriptor_thresh_cpu = 0.115;

    /** \brief Threshold on descriptor distance. */
    double descriptor_thresh_gpu = 0.55;

    /** \brief Flag to screen previously matched landmarks. */
    bool screen_matched_landmarks = true;

    /**
     * \brief the maximum allowed depth difference between a map and query
     * landmark.
     */
    double max_depth_diff = 5.0;

    /** \brief The number of openmp threads to use when matching */
    int parallel_threads = 8;

    /** \brief Match on the GPU or on CPU */
    bool match_on_gpu = false;

    /** \brief When doing GPU matching, the number of KNN matches */
    int match_gpu_knn_match_num = 8;

    /** \brief Whether learned image features are in use instead of SURF*/
    bool use_learned_features = false;

    /** \brief Visualization flag. */
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);

  };
  
  MelMatcherModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}



 protected:
  /**
   * \brief Matches a query stereo frame to a map stereo frame and fills in the
   * inlier between them.
   */
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;


 private:
  /** \brief Resets local variables. */
  void reset();

  /**
   * \brief Initialize the matches data structure.
   * \param query_landmarks The collection of query landmarks.
   * \param matches The matches to be formatted.
   */
  void initializeMatches(const std::vector<LandmarkFrame> &query_landmarks,
                         std::vector<RigMatches> &matches);

  /**
   * \brief Matches the current landmarks across multiple experiences.
   * \param qdata The query data Cache.
   * \param graph The STPG.
   */
  void matchAcrossExperiences(CameraQueryCache &qdata,
                              const tactic::Graph::ConstPtr &graph);

  /**
   * \brief Finds matches between the query landmarks and map landmarks found
   * in a given vertex.
   * \param qdata The query cache data.
   * \param vertex vertex the current vertex.
   */
  void matchVertex(CameraQueryCache &qdata, tactic::Vertex::Ptr vertex);

  /**
   * \brief Finds matches between query and map for a given channel.
   * \param qdata The query data cache.
   * \param channel_id the Identification of the current channel
   * (vertex,rig,channel). \param map_channel_lm The list of landmarks in the
   * map for the given channel.
   */
  void matchChannel(CameraQueryCache &qdata, const LandmarkId &channel_id,
                    const vtr_vision_msgs::msg::ChannelLandmarks &map_channel_lm);

  /**
   * \brief Finds matches between query and map for a given channel while
   * leveraging the GPU
   * \param qdata The query data cache.
   * \param channel_id the Identification of the current channel
   * (vertex,rig,channel). \param map_channel_lm The list of landmarks in the
   * map for the given channel.
   */
  void matchChannelGPU(
      CameraQueryCache &qdata, const LandmarkId &channel_id,
      const vtr_vision_msgs::msg::ChannelLandmarks &map_channel_lm);

  /**
   * \brief Attempts to find a match between a query landmark and a set of map
   * landmarks.
   * \param qdata The query data cache.
   * \param channel_id the Identification of the current channel
   * (vertex,rig,channel).
   * \param q_kp_idx the index of the current query keypoint.
   * \param map_channel_lm The list of landmarks in the map for the given
   * channel.
   */
  int matchQueryKeypoint(
      CameraQueryCache &qdata, const LandmarkId &channel_id,
      const int &q_kp_idx,
      const vtr_vision_msgs::msg::ChannelLandmarks &map_channel_lm);

  /**
   * \brief Matches a query and map landmark.
   * \param query_lm_info The appearance information for the query landmark.
   * \param lm_info_map The appearance information for the map landmark.
   * \param map_track_length TODO
   * \param query_kp the Query keypoint.
   * \param map_kp the map keypoint.
   * \param query_depth TODO
   * \param map_depth TODO
   * \param lm_track The track of landmarks that match to the map landmark.
   * \return true if the query and map landmarks are a potential match.
   */
  bool potential_match(const cv::KeyPoint &query_lm_info,
                       const vtr_vision_msgs::msg::FeatureInfo &lm_info_map,
                       const int &map_track_length, const cv::Point &query_kp,
                       const Eigen::Vector2d &map_kp, const double &query_depth,
                       const double &map_depth,
                       const vtr_vision_msgs::msg::Match &lm_track);

  /** \brief Algorithm Configuration */
  Config::ConstPtr config_;

  /** \brief Matched flags for the query landmarks. */
  std::vector<bool> query_matched_;

  std::unordered_map<LandmarkId, bool> map_matched_;
  
  common::timing::Stopwatch<> timer_;
  int total_match_count_;

  bool use_tight_pixel_thresh_;

  VTR_REGISTER_MODULE_DEC_TYPE(MelMatcherModule);

};

}  // namespace vision
}  // namespace vtr
