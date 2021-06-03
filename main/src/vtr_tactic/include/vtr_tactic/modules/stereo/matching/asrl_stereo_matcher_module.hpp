#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/visualize.hpp>
#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>

namespace vtr {
namespace tactic {

/**
 * \brief Reject outliers and estimate a preliminary transform
 * \details
 * requires:
 *   qdata.[rig_calibrations, rig_features, candidate_landmarks,
 *          T_sensor_vehicle],
 *   qdata.[map_landmarks, T_r_m_prior, T_sensor_vehicle_map]
 * outputs:
 *   qdata.[raw_matches]
 */
class ASRLStereoMatcherModule : public BaseModule {
 public:
  static constexpr auto static_name = "asrl_stereo_matcher";

  // use a method to predict where pixels have moved
  enum PredictionMethod { none = 0, se3 = 1 };

  struct Config {
    /**
     * \brief Check the laplacian bits are the same when matching two keypoints
     */
    bool check_laplacian_bit;

    /** \brief Check the octaves are the same when matching two keypoints */
    bool check_octave;

    /** \brief Check the responses are the same when matching two keypoints */
    bool check_response;

    /**
     * \brief The minimum ratio between the two responses.
     * \details
     * A value of 1.0 means they must be exactly the same
     * A value of 0.0 means they can be infinitely different
     * A value of about 0.1 is a good place to start
     */
    double min_response_ratio;

    //
    //    /** \brief The maximum distance threshold between a candidate
    //    ///        match, in pixels. This will move with the prediction
    //    ///        method's prediction of pixel location
    //     */
    //
    //    int window_size;

    /// The maximum distance threshold between a candidate match [pixels].
    int matching_pixel_thresh;

    /// The maximum distance threshold between a candidate match [pixels],
    /// when we are very confident.
    int tight_matching_pixel_thresh;

    /// The standard deviation threshold for pose uncertainty to qualify for
    /// tight matching
    double tight_matching_x_sigma;
    double tight_matching_y_sigma;
    double tight_matching_theta_sigma;

    /**
     * \brief Scales the search window to reduce low precision feature search
     * windows, like so:
     * search_window_in_pixels =
     *   window_size*(sqrt(1.0/precision)^window_pow_scale
     */
    double window_pow_scale;

    /**
     * \brief Whether or not to use the precision associated with the keypoint
     * to inflate the window size
     */
    bool use_pixel_variance;

    /** \brief Use an estimate of motion to predict where pixels have moved */
    PredictionMethod prediction_method;

    /** \brief The maximum point depth considered for matches. */
    double max_point_depth;

    /** \brief The maximum Euclidean distance threshold for a match. */
    double descriptor_thresh;

    /** \brief The number of openmp threads to use when matching */
    int parallel_threads;

    /** \brief Whether or not to visualise the matches */
    bool visualize_feature_matches;

    /** \brief How many features are found to constitute a success */
    int min_matches;
  };

  /** \brief Constructor */
  ASRLStereoMatcherModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {
    use_tight_pixel_thresh_ = false;
    force_loose_pixel_thresh_ = false;
  }

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

  /** \brief Perform the feature matching */
  unsigned matchFeatures(QueryCache &qdata, MapCache &mdata,
                         const std::shared_ptr<const Graph> &graph);

 private:
  /**
   * \brief This module matches a query stereo frame to a map stereo frame and
   * fills in the inlier between them.
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Visualization implementation */
  void visualizeImpl(QueryCache &qdata, MapCache &mdata,
                     const Graph::ConstPtr &graph,
                     std::mutex &vis_mtx) override;

  /**
   * \brief Check the keypoint data to see if it's worth doing a descriptor
   * match
   */
  bool checkConditions(const vision::Keypoint &kp_map,
                       const vision::FeatureInfo &lm_info_map,
                       const vision::Keypoint &kp_query,
                       const vision::FeatureInfo &lm_info_qry,
                       const cv::Point &qry_pt, const cv::Point &map_pt);

  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;

  bool use_tight_pixel_thresh_;
  bool force_loose_pixel_thresh_;
};

}  // namespace tactic
}  // namespace vtr
