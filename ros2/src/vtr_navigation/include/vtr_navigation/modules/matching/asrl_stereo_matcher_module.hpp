#pragma once

#include <vtr_navigation/modules/base_module.hpp>
#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>

#if false
#include <asrl/messages/Matches.pb.h>
#endif

namespace vtr {
namespace navigation {

/** \brief Reject outliers and estimate a preliminary transform */
class ASRLStereoMatcherModule : public BaseModule {
 public:
  static constexpr auto type_str_ = "asrl_stereo_matcher";

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

    /** \brief The minimum ratio between the two responses.
     *
     *       A value of 1.0 means they must be exactly the same
     *       A value of 0.0 means they can be infinitely different
     *       A value of about 0.1 is a good place to start
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

    /** \brief Scales the search window to reduce low precision feature search
     * windows, like so:
     * search_window_in_pixels =
     * window_size*(sqrt(1.0/precision)^window_pow_scale
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
  ASRLStereoMatcherModule(std::string name = type_str_) : BaseModule{name} {
    use_tight_pixel_thresh_ = false;
    force_loose_pixel_thresh_ = false;
  };

  /**
   * \brief This module matches a query stereo frame to a map stereo frame and
   * fills in the inlier between them.
   */
  void run(QueryCache &qdata, MapCache &mdata,
           const std::shared_ptr<const Graph> &graph) override;

  /** \brief Perform the feature matching */
  unsigned matchFeatures(QueryCache &qdata, MapCache &mdata,
                         const std::shared_ptr<const Graph> &graph);

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 protected:
  /** \brief Visualization implementation */
  virtual void visualizeImpl(QueryCache &qdata, MapCache &mdata,
                             const std::shared_ptr<const Graph> &,
                             std::mutex &vis_mtx);

 private:
  /**
   * \brief Check the keypoint data to see if it's worth doing a descriptor
   * match
   */
  bool checkConditions(const vtr::vision::Keypoint &kp_map,
                       const vtr::vision::FeatureInfo &lm_info_map,
                       const vtr::vision::Keypoint &kp_query,
                       const vtr::vision::FeatureInfo &lm_info_qry,
                       const cv::Point &qry_pt, const cv::Point &map_pt);

  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;

  bool use_tight_pixel_thresh_;
  bool force_loose_pixel_thresh_;
};

}  // namespace navigation
}  // namespace vtr
