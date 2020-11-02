#include <gtest/gtest.h>

#include <lgmath/se3/Transformation.hpp>
#include <vtr_storage/data_stream_reader.hpp>

#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/image.hpp>
#include <vtr_messages/msg/rig_calibration.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_vision/features/extractor/orb_feature_extractor.hpp>
#include <vtr_vision/geometry/geometry_tools.hpp>
#include <vtr_vision/image_conversions.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_vision/sensors/stereo_transform_model.hpp>
#include <vtr_vision/types.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <random>

namespace fs = std::filesystem;
using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;

// RANSAC solution probability
// given the number of attempted sample iterations, the number of points in the
// sample and the outlier probability, give the probability of finding a
// solution
float probability_of_success(int num_samples, int num_sample_points,
                             float outlier_probability) {
  return (1.0 -
          std::pow(1.0 - std::pow(1.0 - outlier_probability, num_sample_points),
                   num_samples));
}

// get the mean of a vector of type T
template <typename T>
float mean(std::vector<T> vec) {
  double sum = std::accumulate(std::begin(vec), std::end(vec), 0.0);
  return sum / vec.size();
}

// get the std deviation of a vector of type T
template <typename T>
float stddev(std::vector<T> vec) {
  float mn = mean(vec);
  float accum = 0.0;
  std::for_each(std::begin(vec), std::end(vec),
                [&](const T dev) { accum += (dev - mn) * (dev - mn); });

  return sqrt(accum / (vec.size() - 1));
}

TEST(Vision, ransac) {
  fs::path dataset_dir{fs::current_path() / "sample_data"};

  vtr::storage::DataStreamReader<RigImages, RigCalibration> stereo_stream(dataset_dir.string(), "front_xb3");

  // make a random number generator and seed with current time
  std::default_random_engine eng(0);

  // make a uniform distribution the same size as the dataset
  std::uniform_int_distribution<int> dist =
      std::uniform_int_distribution<int>(1, 50);

  // how many RANSAC tests do we want to do?
  unsigned total_samples = 3;

  // try total_samples samples of the dataset with RANSAC
  for (unsigned sample_count = 0; sample_count < total_samples;
       sample_count++) {
    // get a sample index
    int sample = dist(eng);

    vtr_messages::msg::RigCalibration calibration_msg;
    EXPECT_NO_THROW(calibration_msg = stereo_stream.fetchCalibration()->get<RigCalibration>());
    auto rig_calibration = vtr::messages::copyCalibration(calibration_msg);

    // make an orb feature extractor configuration
    vtr::vision::ORBConfiguration extractor_config{};
    extractor_config.num_detector_features_ = 15000;
    extractor_config.num_binned_features_ = 1000;
    extractor_config.scaleFactor_ = 1.8;
    extractor_config.nlevels_ = 4;
    extractor_config.edgeThreshold_ = 8;
    extractor_config.firstLevel_ = 0;
    extractor_config.WTA_K_ = 2;
    extractor_config.scoreType_ = cv::ORB::HARRIS_SCORE;
    extractor_config.patchSize_ = 48;
    extractor_config.x_bins_ = 6;
    extractor_config.y_bins_ = 4;
    extractor_config.upright_ = true;

    // set the configuration for the matcher
    vtr::vision::ASRLFeatureMatcher::Config matcher_config{};
    matcher_config.stereo_y_tolerance_ = 2.0;
    matcher_config.stereo_x_tolerance_min_ = 0;
    matcher_config.stereo_x_tolerance_max_ = 200;
    matcher_config.descriptor_match_thresh_ = 0.2;
    matcher_config.stereo_descriptor_match_thresh_ = 0.2;

    matcher_config.check_octave_ = true;
    matcher_config.check_response_ = true;
    matcher_config.min_response_ratio_ = 0.2;
    matcher_config.scale_x_tolerance_by_y_ = true;
    matcher_config.x_tolerance_scale_ = 768;
    extractor_config.stereo_matcher_config_ = matcher_config;

    // create the extractor
    vtr::vision::OrbFeatureExtractor extractor;
    extractor.initialize(extractor_config);

    // Get the first message
    bool continue_stream = true;
    auto data_msg_prev = stereo_stream.readAtIndex(sample);
    continue_stream &= (data_msg_prev != nullptr);
    // make sure we have data
    ASSERT_TRUE(continue_stream);

    // extract the images from the previous data message
    auto ros_rig_images_prev_rgb = data_msg_prev->get<RigImages>();
    vtr::vision::RigImages rig_images_prev_rgb =
        vtr::messages::copyImages(ros_rig_images_prev_rgb);

    // the images are in RGB, we need to convert them to grayscale
    // and put them in a new queue because we don't want to extract features for
    // the rgb images
    vtr::vision::RigImages rig_images_prev;
    rig_images_prev.name = rig_images_prev_rgb.name;
    rig_images_prev.channels.emplace_back(
        vtr::vision::RGB2Grayscale(rig_images_prev_rgb.channels[0]));

    // extract the desired features
    vtr::vision::RigFeatures rig_features_prev =
        extractor.extractRigFeatures(rig_images_prev, true);
    // make a new set of rig landmarks
    vtr::vision::RigLandmarks rig_landmarks_prev;
    rig_landmarks_prev.name = rig_images_prev.name;

    // Now triangulate all the points from the features for the previous rig
    for (const vtr::vision::ChannelFeatures &channel :
         rig_features_prev.channels) {
      // set up the candidate landmarks for this channel.
      rig_landmarks_prev.channels.emplace_back(vtr::vision::ChannelLandmarks());
      auto &landmarks = rig_landmarks_prev.channels.back();
      landmarks.name = channel.name;

      // if this channel has no features, then go to the next
      if (channel.cameras.empty()) {
        continue;
      }

      // copy the descriptor info from the feature.
      landmarks.appearance.descriptors = channel.cameras[0].descriptors.clone();
      landmarks.appearance.feat_infos = channel.cameras[0].feat_infos;
      landmarks.appearance.keypoints = channel.cameras[0].keypoints;
      landmarks.appearance.feat_type = channel.cameras[0].feat_type;
      landmarks.appearance.name = channel.cameras[0].name;

      // Iterate through the observations of the landmark from each camera and
      // triangulate.
      auto num_cameras = channel.cameras.size();
      auto num_keypoints = channel.cameras[0].keypoints.size();
      landmarks.points.resize(3, num_keypoints);
      for (uint32_t keypoint_idx = 0; keypoint_idx < num_keypoints;
           ++keypoint_idx) {
        std::vector<cv::Point2f> keypoints;
        for (uint32_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
          keypoints.emplace_back(
              channel.cameras[camera_idx].keypoints[keypoint_idx].pt);
        }
        // set up the landmarks.
        landmarks.points.col(keypoint_idx) =
            vtr::vision::triangulateFromRig(rig_calibration, keypoints);
      }
    }

    // get the next message
    auto data_msg_next = stereo_stream.readAtIndex(sample + 5);
    continue_stream &= (data_msg_prev != nullptr);
    // make sure we have data
    ASSERT_TRUE(continue_stream);

    // extract the images from the next data message
    auto ros_rig_images_next_rgb = data_msg_next->get<RigImages>();
    vtr::vision::RigImages rig_images_next_rgb =
        vtr::messages::copyImages(ros_rig_images_next_rgb);

    // the images are in RGB, we need to convert them to grayscale
    // and put them in a new queue because we don't want to extract features for
    // the rgb images
    vtr::vision::RigImages rig_images_next;
    rig_images_next.name = rig_images_next_rgb.name;
    rig_images_next.channels.emplace_back(
        vtr::vision::RGB2Grayscale(rig_images_next_rgb.channels[0]));

    // extract the desired features
    vtr::vision::RigFeatures rig_features_next =
        extractor.extractRigFeatures(rig_images_next, true);

    // make a new set of rig landmarks
    vtr::vision::RigLandmarks rig_landmarks_next;
    rig_landmarks_next.name = rig_images_next.name;

    // Now triangulate all the points from the features for the next rig

    // for each channel
    for (const vtr::vision::ChannelFeatures &channel :
         rig_features_next.channels) {
      // set up the candidate landmarks for this channel.
      rig_landmarks_next.channels.emplace_back(vtr::vision::ChannelLandmarks());
      auto &landmarks = rig_landmarks_next.channels.back();
      landmarks.name = channel.name;

      // if this channel has no features, then go to the next
      if (channel.cameras.empty()) {
        continue;
      }

      // copy the descriptor info from the feature.
      landmarks.appearance.descriptors = channel.cameras[0].descriptors.clone();
      landmarks.appearance.feat_infos = channel.cameras[0].feat_infos;
      landmarks.appearance.keypoints = channel.cameras[0].keypoints;
      landmarks.appearance.feat_type = channel.cameras[0].feat_type;
      landmarks.appearance.name = channel.cameras[0].name;

      // Iterate through the observations of the landmark from each camera and
      // triangulate.
      auto num_cameras = channel.cameras.size();
      auto num_keypoints = channel.cameras[0].keypoints.size();
      landmarks.points.resize(3, num_keypoints);
      for (uint32_t keypoint_idx = 0; keypoint_idx < num_keypoints;
           ++keypoint_idx) {
        std::vector<cv::Point2f> keypoints;
        for (uint32_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
          keypoints.emplace_back(
              channel.cameras[camera_idx].keypoints[keypoint_idx].pt);
        }
        // set up the landmarks.
        landmarks.points.col(keypoint_idx) =
            vtr::vision::triangulateFromRig(rig_calibration, keypoints);
      }
    }

    // now match the features and landmarks
    vtr::vision::ASRLFeatureMatcher matcher(matcher_config);
    float window_size = 400;
    // now match the features just from the base frames
    vtr::vision::SimpleMatches close_matches = matcher.matchFeatures(
        rig_features_prev.channels[0].cameras[0],
        rig_features_next.channels[0].cameras[0], window_size);

    // Set up RANSAC

    // make a stereo ransac model
    vtr::vision::StereoTransformModel::Ptr ransac_model =
        std::make_shared<vtr::vision::StereoTransformModel>();
    ASSERT_NE(ransac_model, nullptr);

    // set up the measurement variance
    Eigen::Matrix<double, 2, Eigen::Dynamic> inv_r_matrix;
    unsigned num_points_prev =
        rig_features_prev.channels[0].cameras[0].keypoints.size();
    inv_r_matrix.resize(2, 2 * num_points_prev);
    for (unsigned i = 0; i < num_points_prev; i++) {
      inv_r_matrix.block(0, 2 * i, 2, 2) = rig_features_prev.channels[0]
                                               .cameras[0]
                                               .feat_infos[i]
                                               .covariance.inverse();
    }
    ransac_model->setMeasurementVariance(inv_r_matrix);

    // set the 3D points
    ransac_model->setPoints(&rig_landmarks_prev.channels[0].points,
                            &rig_landmarks_next.channels[0].points);

    // set the calibration
    auto extrinsic = rig_calibration.extrinsics[1];
    auto baseline = -extrinsic.matrix()(0, 3);
    auto intrinsic = rig_calibration.intrinsics[0];
    ransac_model->setCalibration(intrinsic, baseline);

    // Create a feature mask for close features (to be favoured in RANSAC
    // sampling)
    int num_points_next =
        rig_features_next.channels[0].cameras[0].keypoints.size();
    std::vector<bool> mask(num_points_next);
    for (unsigned int i = 0; i < mask.size(); ++i) {
      mask[i] = rig_landmarks_next.channels[0].points(2, i) <
                10000;  // close points are within ransac_mask_depth metres
    }

    // Set up the verifier
    int ransac_mask_depth_inlier_count = 0;
    auto verifier = std::make_shared<vtr::vision::VerifySampleSubsetMask>(
        ransac_mask_depth_inlier_count, mask);

    // set up the sampler
    auto sampler = std::make_shared<vtr::vision::BasicSampler>(verifier);

    double sigma = 3.5;
    double threshold = 10.0;
    int iterations = 2000;
    double early_stop_ratio = 1.0;
    double early_stop_min_inliers = 200;

    // make the ransac estimator
    vtr::vision::VanillaRansac<Eigen::Matrix4d> ransac(
        sampler, sigma, threshold, iterations, early_stop_ratio,
        early_stop_min_inliers);
    // give it the ransac model
    ransac.setCallback(ransac_model);

    // now attempt to run ransac repeatedly
    int num_tries = 10;

    // keep a record of the inlier count
    std::vector<int> inliers_count;
    inliers_count.reserve(num_tries);

    for (int ii = 0; ii < num_tries; ii++) {
      // Make containers for the return results
      Eigen::Matrix4d solution;
      vtr::vision::SimpleMatches inliers;

      // Run RANSAC
      EXPECT_TRUE(ransac.run(close_matches, &solution, &inliers));

      inliers_count.push_back(inliers.size());
    }

    // probability that a point is an outlier
    // this is deliberately conservative
    float p = 0.67;

    // number of points used in calculating solution
    int nn = 6;

    // how many successes are we likely to get?
    float success_prob = probability_of_success(iterations, nn, p);

    // sort the inlier counts
    std::sort(inliers_count.begin(), inliers_count.end());

    // get the maximum inlier count
    unsigned inlier_max = inliers_count.back();

    // count how many are less than 80% of the maximum
    // any that fit this criteria are considered a failure
    unsigned fail_count = 0;
    for (float i : inliers_count) {
      if (i < 0.8 * inlier_max) fail_count++;
    }

    // make sure the number of fails is less than or equal to the expected
    EXPECT_LE(fail_count, std::ceil((1.0 - success_prob) * num_tries));
  }
}
