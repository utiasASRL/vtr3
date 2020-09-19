#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vtr_vision/types.hpp>
#include <future>

namespace vtr {
namespace vision {

/////////////////////////////////////////////////////////////////////////
/// @class Feature Matches Base Class for the asynchronous visual feature
///        matcher.
/// @details This is the base class that manages matching of visual features
///          from an image pipeline.
/////////////////////////////////////////////////////////////////////////
class ASRLFeatureMatcher {
 public:

  enum struct CheckType {HOMOGRAPHY = 0, EPIPOLE = 1};
  struct EpipoleHelper {
    // this stores the line equation
    Eigen::Vector3d l;
    // this caches a common function result [sqrt(a^2_b^2)]
    float a2b2;
    // this caches a common function result [sqrt(a^2_b^2)]
    float sqrta2b2;
  };
  struct Config {
    // the allowable difference in y value
    // for the compared stereo keypoint positions (pixels)
    double stereo_y_tolerance_;
    // the allowable minimum difference in
    // x value for the compared stereo keypoint
    // positions (left - right) (pixels)
    double stereo_x_tolerance_min_;
    // the allowable maximum difference in
    // x value for the compared stereo keypoint
    // positions (left - right) (pixels)
    double stereo_x_tolerance_max_;
    // the minimum difference in
    // stereo descriptor matching value to allow
    // a match (must be between 0 and 1)
    // 1 is perfect match, 0 is worst
    double descriptor_match_thresh_;
    // the minimum difference in
    // descriptor matching value to allow
    // a match (must be between 0 and 1)
    // 1 is perfect match, 0 is worst
    double stereo_descriptor_match_thresh_;
    // check the laplacian bits are the same when matching two keypoints
    bool check_laplacian_bit_;
    // check the octaves are the same when matching two keypoints
    bool check_octave_;
    // check the responses are the same when matching two keypoints
    bool check_response_;
    // the minimum ratio between the two responses.
    // a value of 1.0 means they must be exactly the same
    // a value of 0.0 means they can be infinitely different
    // a value of about 0.1 is a good place to start
    double min_response_ratio_;
    // if the data approximates a ground vehicle
    // looking across a plane, then distant points with
    // low disparity will likely be at the top of the image
    // and close points with high disparity will likely be at
    // the bottom. By setting scale_x_tolerance_by_y_ true,
    // the stereo matcher will scale stereo_x_tolerance_max_
    // by the left image y value divided by x_tolerance_scale_
    // such that for each keypoint stereo_x_tolerance_max_ =
    // stereo_x_tolerance_max_*kpt.y/config_.x_tolerance_scale_
    bool scale_x_tolerance_by_y_;
    double x_tolerance_scale_;
    // number of threads to use when matching
    int num_threads_;
  };
  /////////////////////////////////////////////////////////////////////////
  /// @brief Default constructor
  /////////////////////////////////////////////////////////////////////////
  explicit ASRLFeatureMatcher(Config config);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Destructor
  /////////////////////////////////////////////////////////////////////////
  ~ASRLFeatureMatcher();

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the derived feature extractor.
  /// @note this is a pure virtual function and must be implemented by a
  ///       concrete class.
  /////////////////////////////////////////////////////////////////////////
  //virtual void initialize(ASRLFeatureMatcherConfiguration &config)=0;

  /////////////////////////////////////////////////////////////////////////
  /// @brief Matches features contained by frame1 and frame2. No prior used. blocking.
  /////////////////////////////////////////////////////////////////////////
  SimpleMatches matchFeatures(const Features &frame1, const Features &frame2);
  SimpleMatches matchFeatures(const Features &frame1, const Features &frame2, const float &window_size);
  /////////////////////////////////////////////////////////////////////////
  /// @brief Matches features contained by frame1 and frame2 using rectified stereo parameters. blocking.
  /////////////////////////////////////////////////////////////////////////
  SimpleMatches matchStereoFeatures(const Features &frame1, const Features &frame2);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Matches features contained by frame1 and frame2,
  ///       using the homography H_1_2 and search window given by the x and y window sizes.
  ///       Assumes the depth of the 3D points being tracked are much greater than the size
  ///       of the rotation. blocking.
  /////////////////////////////////////////////////////////////////////////
  SimpleMatches matchFeatures(const Features &frame1, const Features &frame2,
                              const Eigen::Matrix3d &H_2_1,
                              const float &x_window_min,
                              const float &x_window_max,
                              const float &y_window_size,
                              const CheckType type = CheckType::HOMOGRAPHY);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Checks the parameters of the two descriptors and supporting info
  /// according to the config and returns true if all conditions are met
  /////////////////////////////////////////////////////////////////////////
  bool checkConditions(const Keypoint &kp1,
                       const FeatureInfo &fi1,
                       const Keypoint &kp2,
                       const FeatureInfo &fi2,
                       const EpipoleHelper &eh,
                       const float &x_window_size_min,
                       const float &x_window_size_max,
                       const float &y_window_size,
                       const CheckType &type);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Checks that the epipolar distance is valid
  /////////////////////////////////////////////////////////////////////////
  bool checkEpipole(const Keypoint &kp1,
                    const FeatureInfo &fi1,
                    const Keypoint &kp2,
                    const FeatureInfo &fi2,
                    const EpipoleHelper &eh,
                    const float &x_window_size_min,
                    const float &x_window_size_max,
                    const float &y_window_size);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Compares the distance between two brief descriptors
  /// @param[in] two pointers to unsigned chars of size 'size'
  /// @return The distance between the two descriptors. 0.0 is perfect match,
  ///       1.0 is completely different.
  /////////////////////////////////////////////////////////////////////////
  static float briefmatch(const unsigned char *d1, const unsigned char *d2, unsigned size);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Compares the distance between two surf descriptors
  /// @param[in] two pointers to floats of size 'size'
  /// @return The distance between the two descriptors. 0.0 is perfect match,
  ///       1.0 is completely different.
  /////////////////////////////////////////////////////////////////////////
  static float surfmatch(const float *d1, const float *d2, unsigned size);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Compares the distance between two descriptors
  /// @param[in] two pointers to the appropriate type (see above) of size 'size'
  /// @return The distance between the two descriptors. 0.0 is perfect match,
  ///       1.0 is completely different, -1.0 is an invalid feature type
  /////////////////////////////////////////////////////////////////////////
  static float distance(const void * d1, const void * d2, const FeatureType &feat_type);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Adds a stereo extraction task for the input set of images, non-blocking
  /// @note this is a pure virtual function and must be implemented by a
  ///       concrete class.
  /////////////////////////////////////////////////////////////////////////
  void addMatchingTask(const Features &frame1, const Features &frame2);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Adds a stereo extraction task for the input set of images, non-blocking
  /// @note this is a pure virtual function and must be implemented by a
  ///       concrete class.
  /////////////////////////////////////////////////////////////////////////
  void addStereoMatchingTask(const Features &frame1, const Features &frame2);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Grabs the oldest completed frame from the asynchronous pipe.
  /////////////////////////////////////////////////////////////////////////
  SimpleMatches getOldestMatchTask();

  /////////////////////////////////////////////////////////////////////////
  /// @brief Grabs the oldest completed frame from the asynchronous pipe.
  /////////////////////////////////////////////////////////////////////////
  SimpleMatches getOldestStereoMatchTask();

 protected:

  /////////////////////////////////////////////////////////////////////////
  /// @brief A queue of future frames.
  /////////////////////////////////////////////////////////////////////////
  std::queue<std::future<SimpleMatches>> future_matches;

  /////////////////////////////////////////////////////////////////////////
  /// @brief A queue of future frames.
  /////////////////////////////////////////////////////////////////////////
  std::queue<std::future<SimpleMatches>> future_stereomatches;

  /////////////////////////////////////////////////////////////////////////
  /// @brief Matcher configuration
  /////////////////////////////////////////////////////////////////////////
  Config config_;
};

}}
