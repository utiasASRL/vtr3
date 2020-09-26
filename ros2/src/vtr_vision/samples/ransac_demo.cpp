// lgmath includes
#include <lgmath/se3/Transformation.hpp>

// robochunk includes
//#include <robochunk/base/DataStream.hpp>
//#include <robochunk_msgs/MessageBase.pb.h>
//#include <robochunk/util/fileUtils.hpp>
#include <vtr_storage/data_stream_reader.hpp>

//  messages
#include <vtr_messages/msg/image.hpp>
#include <vtr_messages/msg/channel_images.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/rig_calibration.hpp>

// vtr3 includes
#include <vtr_vision/features/extractor/feature_extractor_factory.hpp>
#include <vtr_vision/features/extractor/orb_feature_extractor.hpp>
#include <vtr_vision/image_conversions.hpp>
#include <vtr_vision/geometry/geometry_tools.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/sensors/stereo_transform_model.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_logging/logging_init.hpp>

// stl includes
#include <memory>
#include <fstream>

// opencv includes
#include <opencv2/opencv.hpp>

using RigImages = vtr_messages::msg::RigImages;

cv::Mat CreateMatchView(vtr::vision::ChannelImages &cameras_prev, vtr::vision::ChannelImages &cameras_next) {
  // Create the image
  cv::Mat matchView = cv::Mat(cameras_prev.cameras[0].data.rows,
                              cameras_prev.cameras[0].data.cols*2,
                              CV_8UC3);

  int offset1 = (matchView.cols*3)/2;
  for(int row = 0; row < cameras_prev.cameras[0].data.rows; row++)
  {
    for(int col = 0; col < cameras_prev.cameras[0].data.cols; col++)
    {
      int pixelIdx = row*matchView.cols*3+col*3;
      if(cameras_prev.cameras[0].data.rows > 0) {
        //Top Left
        matchView.data[pixelIdx] =   cameras_prev.cameras[0].data.data[row*cameras_prev.cameras[0].data.cols+col];
        matchView.data[pixelIdx+1] = matchView.data[pixelIdx];
        matchView.data[pixelIdx+2] = matchView.data[pixelIdx];

        //Top Right
        matchView.data[pixelIdx+offset1] = cameras_next.cameras[0].data.data[row*cameras_prev.cameras[0].data.cols+col];
        matchView.data[pixelIdx+offset1+1] = matchView.data[pixelIdx+offset1];
        matchView.data[pixelIdx+offset1+2] = matchView.data[pixelIdx+offset1];
      }
    }
  }
  // Fill in matches
  return matchView;
}

bool visualizeMatches(vtr::vision::ChannelFeatures &features_prev, vtr::vision::ChannelFeatures &features_next,
                      vtr::vision::ChannelLandmarks &landmarks_prev, vtr::vision::ChannelLandmarks &,
                      vtr::vision::ChannelImages &cameras_prev, vtr::vision::ChannelImages &cameras_next,
                      vtr::vision::SimpleMatches &inliers, cv::Scalar) {

  cv::Mat matchView = CreateMatchView(cameras_prev, cameras_next);

  //cv::Scalar trackColor(203,201,40);
  cv::Scalar distantColor(0,0,255);
  int far_count = 0;
  for(auto & inlier : inliers) {
    cv::KeyPoint kp_prev = features_prev.cameras[0].keypoints[inlier.first];
    cv::KeyPoint kp_curr = features_next.cameras[0].keypoints[inlier.second];
    kp_prev.pt.x += cameras_prev.cameras[0].data.cols;
    kp_curr.pt.x += cameras_prev.cameras[0].data.cols;
    cv::Scalar color;
    //if(features_prev.cameras[0].keypoints[inliers[idx].first].pt.x - features_next.cameras[0].keypoints[inliers[idx].first].pt.x < 0.0) {
    int blue = 0;
    int red = std::max(0,255-20*(int)landmarks_prev.points(2,inlier.first));
    int green = std::min(255,20*(int)landmarks_prev.points(2,inlier.first));
    cv::Scalar kpColor(blue,green,red);
    color = kpColor;
    //++far_count;
    //} else {
    //    color = trackColor;
    //}
    cv::line(matchView,kp_prev.pt,kp_curr.pt,color,2);
    cv::circle(matchView, kp_prev.pt, 4, color);
  }
  cv::imshow("VO Matches",matchView);
  char ret = cv::waitKey(1);
  double ratio = static_cast<double>(far_count)/inliers.size();
  if(ratio > .75 || inliers.size()-far_count < 10) {
    cv::waitKey(1000);
  } else {
    cv::waitKey(1);
  }
  return (ret == 'q');
}


////////////////////////////////////////////////////////////////////////////////////
/// @brief Demonstration of Simple Stereo Visual Odometry uisng the vtr::pose_graph
///        structure
////////////////////////////////////////////////////////////////////////////////////
int main(int, char **) {

  LOG(INFO) << "Starting RANSAC demo";

  vtr::storage::DataStreamReader<RigImages> stereo_stream("/home/ben/wawa", "cakaw");

  vtr::storage::VTRMessage calibration_msg;
  vtr::vision::RigCalibration rig_calibration;
  // Check out the calibration
  if (stereo_stream.fetchCalibration(calibration_msg) == true) {     //todo (group): figure out how to store calibration
    // Extract the intrinsic params out of the bsae message
    std::shared_ptr<vtr_messages::msg::RigCalibration>
        calibration = calibration_msg.extractSharedPayload<vtr_messages::msg::RigCalibration>();
    if (calibration != nullptr) {
      printf("received camera calibration!\n");
      rig_calibration = vtr::messages::copyCalibration(*calibration.get());
    } else {
      printf("ERROR: intrinsic params is not the correct type: (actual: %s \n",
             calibration_msg.header().type_name().c_str());
      return -1;
    }
  } else {
    printf("ERROR: Could not read calibration message!\n");
  }

  // make an orb feature extractor configuraiton
  vtr::vision::ORBConfiguration extractor_config{};
  extractor_config.num_detector_features_ = 15000;
  extractor_config.num_binned_features_ = 1000;
  extractor_config.scaleFactor_ = 1.8;
  extractor_config.nlevels_ = 4;
  extractor_config.edgeThreshold_ = 8;
  extractor_config.firstLevel_ = 0;
  extractor_config.WTA_K_ = 2;
  extractor_config.scoreType_ = cv::ORB::HARRIS_SCORE;
  extractor_config.patchSize_ = 64;
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

  // Seek to an absolute index
  int idx = 3000;

  // Get the first message
  bool continue_stream = true;
  auto data_msg_prev = stereo_stream.readAtIndexRange(idx, idx);   //hacky way to check if messages still in bag
  continue_stream &= !data_msg_prev->empty();

  if (continue_stream) {

    // extract the images from the previous data message
    auto ros_rig_images_prev_rgb = (*data_msg_prev)[0]->get<RigImages>();
    vtr::vision::RigImages rig_images_prev_rgb = vtr::messages::copyImages(ros_rig_images_prev_rgb);

    // the images are in RGB, we need to convert them to grayscale
    // and put them in a new queue because we don't want to extract features for the rgb images
    vtr::vision::RigImages rig_images_prev;
    rig_images_prev.name = rig_images_prev_rgb.name;
    rig_images_prev.channels.emplace_back(vtr::vision::RGB2Grayscale(rig_images_prev_rgb.channels[0]));

    // extract the desired features
    vtr::vision::RigFeatures rig_features_prev = extractor.extractRigFeatures(rig_images_prev, true);

    std::cout << "final extracted features: " << rig_features_prev.channels[0].cameras[0].keypoints.size() << std::endl;

    // make a new set of rig landmarks
    vtr::vision::RigLandmarks rig_landmarks_prev;
    rig_landmarks_prev.name = rig_images_prev.name;

    // Now triangulate all the points from the features for the previous rig

    // for each channel
    for (const vtr::vision::ChannelFeatures &channel : rig_features_prev.channels) {

      // set up the candidate landmarks for this channel.
      rig_landmarks_prev.channels.emplace_back(vtr::vision::ChannelLandmarks());
      auto &landmarks = rig_landmarks_prev.channels.back();
      landmarks.name = channel.name;

      // if this channel has no features, then go to the next
      if (channel.cameras.empty()) { continue; }

      // copy the descriptor info from the feature.
      landmarks.appearance.descriptors = channel.cameras[0].descriptors.clone();
      landmarks.appearance.feat_infos = channel.cameras[0].feat_infos;
      landmarks.appearance.keypoints = channel.cameras[0].keypoints;
      landmarks.appearance.feat_type = channel.cameras[0].feat_type;
      landmarks.appearance.name = channel.cameras[0].name;

      // Iterate through the observations of the landmark from each camera and
      // triangulate.
      // TODO (old): This assumes a fully matched rig.
      auto num_cameras = channel.cameras.size();
      auto num_keypoints = channel.cameras[0].keypoints.size();
      landmarks.points.resize(3, num_keypoints);
      for (uint32_t keypoint_idx = 0; keypoint_idx < num_keypoints; ++keypoint_idx) {
        std::vector<cv::Point2f> keypoints;
        for (uint32_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
          keypoints.emplace_back(channel.cameras[camera_idx].keypoints[keypoint_idx].pt);
        }
        // set up the landmarks.
        landmarks.points.col(keypoint_idx) = vtr::vision::triangulateFromRig(rig_calibration, keypoints);
      }
    }

    idx += 5;

    // get the next message
    auto data_msg_next = stereo_stream.readAtIndexRange(idx, idx);   //hacky way to check if messages still in bag
    continue_stream &= !data_msg_prev->empty();

    if (continue_stream) {

      // extract the images from the next data message
      auto ros_rig_images_next_rgb = (*data_msg_next)[0]->get<RigImages>();
      vtr::vision::RigImages rig_images_next_rgb = vtr::messages::copyImages(ros_rig_images_next_rgb);

      // the images are in RGB, we need to convert them to grayscale
      // and put them in a new queue because we don't want to extract features for the rgb images
      vtr::vision::RigImages rig_images_next;
      rig_images_next.name = rig_images_next_rgb.name;
      rig_images_next.channels.emplace_back(vtr::vision::RGB2Grayscale(rig_images_next_rgb.channels[0]));

      // extract the desired features
      vtr::vision::RigFeatures rig_features_next = extractor.extractRigFeatures(rig_images_next, true);

      // make a new set of rig landmarks
      vtr::vision::RigLandmarks rig_landmarks_next;
      rig_landmarks_next.name = rig_images_next.name;

      // Now triangulate all the points from the features for the next rig

      // for each channel
      for (const vtr::vision::ChannelFeatures &channel : rig_features_next.channels) {

        // set up the candidate landmarks for this channel.
        rig_landmarks_next.channels.emplace_back(vtr::vision::ChannelLandmarks());
        auto &landmarks = rig_landmarks_next.channels.back();
        landmarks.name = channel.name;

        // if this channel has no features, then go to the next
        if (channel.cameras.empty()) { continue; }

        // copy the descriptor info from the feature.
        landmarks.appearance.descriptors = channel.cameras[0].descriptors.clone();
        landmarks.appearance.feat_infos = channel.cameras[0].feat_infos;
        landmarks.appearance.keypoints = channel.cameras[0].keypoints;
        landmarks.appearance.feat_type = channel.cameras[0].feat_type;
        landmarks.appearance.name = channel.cameras[0].name;

        // Iterate through the observations of the landmark from each camera and
        // triangulate.
        // TODO (old): This assumes a fully matched rig.
        auto num_cameras = channel.cameras.size();
        auto num_keypoints = channel.cameras[0].keypoints.size();
        landmarks.points.resize(3, num_keypoints);
        for (uint32_t keypoint_idx = 0; keypoint_idx < num_keypoints; ++keypoint_idx) {
          std::vector<cv::Point2f> keypoints;
          for (uint32_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
            keypoints.emplace_back(channel.cameras[camera_idx].keypoints[keypoint_idx].pt);
          }
          // set up the landmarks.
          landmarks.points.col(keypoint_idx) = vtr::vision::triangulateFromRig(rig_calibration, keypoints);
        }
      }

      // now match the features and landmarks

      // make a matcher
      vtr::vision::ASRLFeatureMatcher matcher(matcher_config);

      // make a window size of 400 pixels
      float window_size = 400;
      // now match the features just from the base frames
      vtr::vision::SimpleMatches close_matches = matcher.matchFeatures(rig_features_prev.channels[0].cameras[0],
                                                                       rig_features_next.channels[0].cameras[0],
                                                                       window_size);

      std::cout << "Total feature matches: " << close_matches.size() << std::endl;
      // Set up RANSAC

      // make a stereo ransac model
      vtr::vision::StereoTransformModel::Ptr ransac_model = std::make_shared<vtr::vision::StereoTransformModel>();
      if (ransac_model == nullptr) {
        std::cout << "Model Has Failed!!!" << std::endl;
        exit(-1);
      }

      // set up the measurement variance
      Eigen::Matrix<double, 1, Eigen::Dynamic> inv_r_matrix;
      unsigned num_points_prev = rig_features_prev.channels[0].cameras[0].keypoints.size();
      inv_r_matrix.resize(1, num_points_prev);
      for (unsigned i = 0; i < num_points_prev; i++) {
        inv_r_matrix(0, i) = rig_features_prev.channels[0].cameras[0].feat_infos[i].precision;
      }
      ransac_model->setMeasurementVariance(inv_r_matrix);

      // set the 3D points
      ransac_model->setPoints(&rig_landmarks_prev.channels[0].points, &rig_landmarks_next.channels[0].points);

      // set the calibration
      auto extrinsic = rig_calibration.extrinsics[1];
      auto baseline = -extrinsic.matrix()(0, 3);
      auto intrinsic = rig_calibration.intrinsics[0];
      std::cout << "baseline: " << baseline << std::endl;
      std::cout << "intrinsic: " << std::endl << intrinsic << std::endl;
      std::cout << "extrinsic: " << std::endl << extrinsic << std::endl;

      ransac_model->setCalibration(intrinsic, baseline);

      // Create a feature mask for close features (to be favoured in RANSAC sampling)
      int num_points_next = rig_features_next.channels[0].cameras[0].keypoints.size();
      std::vector<bool> mask(num_points_next);
      for (unsigned int i = 0; i < mask.size(); ++i) {
        mask[i] = rig_landmarks_next.channels[0].points(2, i) < 10000; // close points are within ransac_mask_depth metres
      }

      // Set up the verifier
      int ransac_mask_depth_inlier_count = 0;
      auto verifier = std::make_shared<vtr::vision::VerifySampleSubsetMask>(ransac_mask_depth_inlier_count, mask);

      // set up the sampler
      auto sampler = std::make_shared<vtr::vision::BasicSampler>(verifier);

      // TODO (old): Set up config for the ransac estimator
      double sigma = 3.5;
      double threshold = 5.0;
      int iterations = 1500;
      double early_stop_ratio = 1.0;
      double early_stop_min_inliers = 1000;

      // make the ransac estimator
      vtr::vision::VanillaRansac<Eigen::Matrix4d> ransac(sampler,
                                                         sigma,
                                                         threshold,
                                                         iterations,
                                                         early_stop_ratio,
                                                         early_stop_min_inliers);

      // give it the ransac model
      ransac.setCallback(ransac_model);

      bool breakout = false;
      // now attempt to run ransac repeatedly
      for (int ii = 0; ii < 500 && !breakout; ii++) {

        // Make containers for the return results
        Eigen::Matrix4d solution;
        vtr::vision::SimpleMatches inliers;

        // Run RANSAC
        if (ransac.run(close_matches, &solution, &inliers) == 0) {
          LOG(WARNING) << "RANSAC returned 0 inliers! A suitable transformation could not be found!";
        } else {
          std::cout << "Matches: " << close_matches.size() << ", Inliers: " << inliers.size() << std::endl;
          cv::Scalar ransacColor(203, 201, 40);
          breakout = visualizeMatches(rig_features_prev.channels[0], rig_features_next.channels[0],
                                      rig_landmarks_prev.channels[0], rig_landmarks_next.channels[0],
                                      rig_images_prev.channels[0], rig_images_next.channels[0],
                                      inliers, ransacColor);
        }

        std::cout << "Transform: " << std::endl;
        std::cout << solution << std::endl;

//    ///////////////// Estimate Transform and plot ////////////////////////////
//    //std::cout << "Calling RANSAC..." << std::endl;
//    avm::MatchList inliers;
//    lgmath::se3::Transformation initial_transform = detectOutliers(previous_features,
//            current_features, matches, inliers, intrinsics, baseline, config);
//
//    // visualise the matches
//    cv::Scalar matchColor(40,201,203);
//    //breakout = visualizeMatches(previous_features,current_features,matches, matchColor);
//    cv::Scalar ransacColor(203,201,40);
//    if(config.show_vo_tracks) {
//        breakout = visualizeMatches(previous_features,current_features,inliers, ransacColor);
//    }
//
//    Eigen::Matrix<double,6,1> vec = initial_transform.vec();

      }

      // clean up the data
//      data_msg_prev.Clear();
//      data_msg_next.Clear();
    }
  }
  // exit cleanly
  exit(0);
}
