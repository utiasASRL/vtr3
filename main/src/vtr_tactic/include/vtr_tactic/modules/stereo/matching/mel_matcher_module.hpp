#pragma once

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_messages/msg/matches.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {
namespace stereo {

/**
 * \brief A module that matches the current live view to a multi-experience
 * map.
 * \details
 * requires:
 *   qdata.[T_sensor_vehicle, rig_names]
 *   qdata.[map_id, T_r_m_prior, T_sensor_vehicle_map, map_landmarks,
 *          localization_map, landmark_offset_map, projected_map_points,
 *          migrated_points_3d, migrated_validity]
 * outputs:
 *   qdata.[raw_matches]
 */
class MelMatcherModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "mel_matcher";

  /** \brief Configuration */
  struct Config {
    /** \brief The target number of matches to early exit */
    int target_match_count;

    /** \brief The minimum number of matches needed. */
    int min_match_count;

    /** \brief The minimum length of feature tracks on map landmarks. */
    int min_track_length;

    /** \brief The maximum depth of a map landmark. */
    double max_landmark_depth;

    /**
     * \brief The maximum distance threshold between a candidate match
     * [pixels].
     */
    int matching_pixel_thresh;

    /**
     * \brief  The maximum distance threshold between a candidate match
     * [pixels], when we are very confident.
     */
    int tight_matching_pixel_thresh;

    /**
     * \brief The standard deviation threshold for pose uncertainty to qualify
     * for tight matching
     */
    double tight_matching_x_sigma;
    double tight_matching_y_sigma;
    double tight_matching_theta_sigma;

    /**
     * \brief The minimum ratio between the two responses.
     * A value of 1.0 means they must be exactly the same
     * A value of 0.0 means they can be infinitely different
     * A value of about 0.1 is a good place to start
     */
    double min_response_ratio;

    /**
     * \brief Time allowance for the matching threshold. If the computation
     * time of the algorithm exceeds this number, it will exit with the current
     * found matches.
     */
    double time_allowance;

    /** \brief Threshold on descriptor distance. */
    double descriptor_thresh_cpu;

    /** \brief Threshold on descriptor distance. */
    double descriptor_thresh_gpu;

    /** \brief Flag to screen previously matched landmarks. */
    bool screen_matched_landmarks;

    /**
     * \brief the maximum allowed depth difference between a map and query
     * landmark.
     */
    double max_depth_diff;

    /** \brief Visualization flag. */
    bool visualize;

    /** \brief The number of openmp threads to use when matching */
    int parallel_threads;

    /** \brief Match on the GPU or on CPU */
    bool match_on_gpu;

    /** \brief When doing GPU matching, the number of KNN matches */
    int match_gpu_knn_match_num;
  };

  /** \brief Constructor */
  MelMatcherModule(std::string name = static_name)
      : BaseModule{name}, map_matched_(2000){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 protected:
  /**
   * \brief Matches a query stereo frame to a map stereo frame and fills in the
   * inlier between them.
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

 private:
  /** \brief Resets local variables. */
  void reset();

  /**
   * \brief Initialize the matches data structure.
   * \param query_landmarks The collection of query landmarks.
   * \param matches The matches to be formatted.
   */
  void initializeMatches(const std::vector<LandmarkFrame> &query_landmarks,
                         std::vector<vision::RigMatches> &matches);

  /**
   * \brief Matches the current landmarks across multiple experiences.
   * \param qdata The query data Cache.
   * \param mdata The map data Cache.
   * \param graph The STPG.
   */
  void matchAcrossExperiences(QueryCache &qdata, MapCache &mdata,
                              const std::shared_ptr<const Graph> &graph);

  /**
   * \brief Finds matches between the query landmarks and map landmarks found
   * in a given vertex.
   * \param qdata The query cache data.
   * \param mdata The map cache data.
   * \param vertex vertex the current vertex.
   */
  void matchVertex(QueryCache &qdata, MapCache &mdata, Vertex::Ptr vertex);

  /**
   * \brief Finds matches between query and map for a given channel.
   * \param qdata The query data cache.
   * \param channel_id the Identification of the current channel
   * (vertex,rig,channel). \param map_channel_lm The list of landmarks in the
   * map for the given channel.
   */
  void matchChannel(QueryCache &qdata, const vision::LandmarkId &channel_id,
                    const vtr_messages::msg::ChannelLandmarks &map_channel_lm);

  /**
   * \brief Finds matches between query and map for a given channel while
   * leveraging the GPU
   * \param qdata The query data cache.
   * \param channel_id the Identification of the current channel
   * (vertex,rig,channel). \param map_channel_lm The list of landmarks in the
   * map for the given channel.
   */
  void matchChannelGPU(
      QueryCache &qdata, const vision::LandmarkId &channel_id,
      const vtr_messages::msg::ChannelLandmarks &map_channel_lm);

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
      QueryCache &qdata, const vision::LandmarkId &channel_id,
      const int &q_kp_idx,
      const vtr_messages::msg::ChannelLandmarks &map_channel_lm);

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
                       const vtr_messages::msg::FeatureInfo &lm_info_map,
                       const int &map_track_length, const cv::Point &query_kp,
                       const Eigen::Vector2d &map_kp, const double &query_depth,
                       const double &map_depth,
                       const vtr_messages::msg::Match &lm_track);

  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;

  /** \brief Matched flags for the query landmarks. */
  std::vector<bool> query_matched_;

  std::unordered_map<vision::LandmarkId, bool> map_matched_;
  common::timing::SimpleTimer timer_;
  int total_match_count_;

  bool use_tight_pixel_thresh_;
};

}  // namespace stereo
}  // namespace tactic
}  // namespace vtr