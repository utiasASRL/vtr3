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
 * \file visualize.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/visualize.hpp>

namespace vtr {
namespace vision {
namespace visualize {

cv::Mat setupDisplayImage(cv::Mat input_image) {
  // create a visualization image to draw on.
  cv::Mat display_image;
  if (input_image.type() == CV_8UC1) {
    cv::cvtColor(input_image, display_image, cv::COLOR_GRAY2RGB);
  } else if (input_image.type() == CV_16S) {
    input_image.convertTo(display_image, CV_8U, 255/(48*16.));
  } else {
    display_image = input_image.clone();
  }
  return display_image;
}

std::map<std::string, cv::Mat> setupDisplayImages(CameraQueryCache &qdata,
                                                  std::string suffix) {
  // set up the display images
  std::map<std::string, cv::Mat> display_map;

  // get the rig images
  auto &rigs = *qdata.rig_images;

  for (auto &rig : rigs) {
    for (auto &channel : rig.channels) {
      for (auto &camera : channel.cameras) {
        auto title = rig.name + "/" + channel.name + "/" + camera.name + suffix;
        // create a visualization image to draw on.
        display_map[title] = setupDisplayImage(camera.data);
      }
    }
  }

  return display_map;
}

void showStereoMatches(std::mutex &vis_mtx, CameraQueryCache &qdata,
                       std::string suffix) {
  // check if the required data is in the cache
  if (!qdata.rig_images.is_valid() || !qdata.rig_features.is_valid() ||
      !qdata.candidate_landmarks.is_valid()) {
    return;
  }

  // get a map of images to titles
  auto display_map = setupDisplayImages(qdata, suffix);

  // make a track color
  cv::Scalar trackColor(203, 201, 40);

  // Check to make sure the display_map isn't empty and we have both features
  // AND landmarks.
  if (!display_map.empty() && qdata.rig_features.is_valid() &&
      qdata.candidate_landmarks.is_valid()) {
    auto &features = *qdata.rig_features;
    auto &landmarks = *qdata.candidate_landmarks;

    // Iterate through both features and landmarks for this rig
    auto feature_itr = features.begin();
    auto landmark_itr = landmarks.begin();
    for (; feature_itr != features.end() && landmark_itr != landmarks.end();
         ++feature_itr, ++landmark_itr) {
      // Iterate through each channel.
      auto feature_channel_itr = feature_itr->channels.begin();
      auto landmark_channel_itr = landmark_itr->channels.begin();
      for (; feature_channel_itr != feature_itr->channels.end() &&
             landmark_channel_itr != landmark_itr->channels.end();
           ++feature_channel_itr, ++landmark_channel_itr) {
        if (feature_channel_itr->cameras.empty()) {
          continue;
        }
        // make a display image for this channel
        cv::Mat display_image = cv::Mat(display_map.begin()->second.rows,
                                        display_map.begin()->second.cols *
                                            feature_channel_itr->cameras.size(),
                                        CV_8UC3);

        // Iterate through each camera.
        uint32_t i = 0;
        for (auto feature_camera_itr = feature_channel_itr->cameras.begin();
             feature_camera_itr != feature_channel_itr->cameras.end();
             feature_camera_itr++) {
          // set up the title
          auto title = feature_itr->name + "/" + feature_channel_itr->name +
                       "/" + feature_camera_itr->name + suffix;
          auto &temp_image = display_map[title];

          // iterate through the keypoints and visualize
          for (uint32_t idx = 0; idx < feature_camera_itr->keypoints.size();
               ++idx) {
            // get the current keypoint
            cv::Point2f &curr = feature_camera_itr->keypoints[idx].pt;

            // draw the connection to the next camera
            if (feature_camera_itr != feature_channel_itr->cameras.end() - 1) {
              cv::Point2f &next = (feature_camera_itr + 1)->keypoints[idx].pt;
              cv::line(temp_image, curr, next, trackColor, 1);
            }
            // draw the connection to the previous camera
            if (feature_camera_itr != feature_channel_itr->cameras.begin()) {
              cv::Point2f &prev = (feature_camera_itr - 1)->keypoints[idx].pt;
              cv::line(temp_image, prev, curr, trackColor, 1);
            }

            // now draw the keypoint with color determined by depth
            const auto &point = landmark_channel_itr->points.col(idx);
            int blue = 0;
            int red = std::max(0, 255 - 20 * (int)point(2));
            int green = std::min(255, 20 * (int)point(2));
            cv::Scalar kpColor(blue, green, red);
            cv::circle(temp_image, curr, 4, kpColor);
          }  // end for keypoints

          // copy to the display image
          cv::Mat sub(display_image,
                      cv::Rect(temp_image.cols * i, 0, temp_image.cols,
                               temp_image.rows));
          temp_image.copyTo(sub);
          i++;

        }  // end for camera

        // make a title minus the camera names
        auto title =
            feature_itr->name + "/" + feature_channel_itr->name + suffix;

        // show the images
        {
          std::lock_guard<std::mutex> lock(vis_mtx);
          cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
          cv::imshow(title, display_image);
        }
      }  // end for channel
    }    // end for rig
    {
      std::lock_guard<std::mutex> lock(vis_mtx);
      cv::waitKey(1);
    }
  }  // end if valid
}

void showDisparity(std::mutex &vis_mtx, CameraQueryCache &qdata, std::string suffix) {
  // check if the required data is in the cache
  if (!qdata.rig_images.is_valid()) return;

  auto &rigs = *qdata.rig_images;

  for (auto &rig : rigs) {
    for (auto &channel : rig.channels) {
      if (channel.name == "disparity") {
        for (auto &camera : channel.cameras) {
          
          auto title = rig.name + "/" + channel.name + "/" + camera.name + suffix;
          // create a visualization image to draw on.
          cv::Mat display_image = setupDisplayImage(camera.data);
   
          // show the images
          {
            std::lock_guard<std::mutex> lock(vis_mtx);
            cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::imshow(title, display_image);
          }

        }  // end for camera
      }
    }    // end for channel
  }      // end for rig
  {
    std::lock_guard<std::mutex> lock(vis_mtx);
    cv::waitKey(1);
  }
}

void showRawFeatures(std::mutex &vis_mtx, CameraQueryCache &qdata,
                     std::string suffix) {
  // check if the required data is in the cache
  if (!qdata.rig_images.is_valid() || !qdata.rig_features.is_valid()) return;

  // get a map of images to titles
  auto display_map = setupDisplayImages(qdata, suffix);

  auto &features = *qdata.rig_features;

  auto features_itr = features.begin();

  // iterate through each rig (for landmarks, matches and features)
  for (; features_itr != features.end(); ++features_itr) {
    // iterate through each channel
    for (auto feature_channel_itr = features_itr->channels.begin();
         feature_channel_itr != features_itr->channels.end();
         ++feature_channel_itr) {
      if (feature_channel_itr->cameras.empty()) continue;
      // iterate through each camera
      auto feature_camera_itr = feature_channel_itr->cameras.begin();
      for (; feature_camera_itr != feature_channel_itr->cameras.end();
           feature_camera_itr++) {
        // set up the title
        auto title = features_itr->name + "/" + feature_channel_itr->name +
                     "/" + feature_camera_itr->name + suffix;
               
        auto &display_image = display_map[title];

        // iterate through the keypoints and visualize
        for (uint32_t idx = 0; idx < feature_camera_itr->keypoints.size();
             ++idx) {
          const auto &keypoint = feature_camera_itr->keypoints[idx];
          const auto &keypoint_info = feature_camera_itr->feat_infos[idx];
          int blue = 255;
          int red = 0;
          int green = 0;
          cv::Scalar kpColor(blue, green, red);
          cv::circle(display_image, keypoint.pt,
                     4.0 / std::sqrt(keypoint_info.precision), kpColor);
        }  // end for keypoints

        // print the number of raw features
        std::ostringstream tostr;
        tostr << feature_camera_itr->keypoints.size();
        cv::putText(display_image,
                    std::string("Features: " + tostr.str()).c_str(),
                    cv::Point(25, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);

        auto title2 = features_itr->name + "/" + feature_channel_itr->name +
                      "/" + feature_camera_itr->name + suffix;

        // show the images
        {
          std::lock_guard<std::mutex> lock(vis_mtx);
          cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
          cv::imshow(title2, display_image);
        }

      }  // end for camera
    }    // end for channel
  }      // end for rig
  {
    std::lock_guard<std::mutex> lock(vis_mtx);
    cv::waitKey(1);
  }
}

void showFeatures(std::mutex &vis_mtx, CameraQueryCache &qdata,
                  std::string suffix) {
  // check if the required data is in the cache
  if (!qdata.rig_images.is_valid() || !qdata.rig_features.is_valid() ||
      !qdata.candidate_landmarks.is_valid()) {
    return;
  }

  // get a map of images to titles
  auto display_map = setupDisplayImages(qdata, suffix);

  auto &features = *qdata.rig_features;
  auto &landmarks = *qdata.candidate_landmarks;

  auto features_itr = features.begin();
  auto landmark_itr = landmarks.begin();

  // iterate through each rig (for landmarks, matches and features)
  for (; features_itr != features.end() && landmark_itr != landmarks.end();
       ++features_itr, ++landmark_itr) {
    // iterate through each channel
    auto feature_channel_itr = features_itr->channels.begin();
    auto landmark_channel_itr = landmark_itr->channels.begin();
    for (; feature_channel_itr != features_itr->channels.end() &&
           landmark_channel_itr != landmark_itr->channels.end();
         ++feature_channel_itr, ++landmark_channel_itr) {
      if (feature_channel_itr->cameras.empty()) {
        continue;
      }

      // iterate through each camera
      auto feature_camera_itr = feature_channel_itr->cameras.begin();
      for (; feature_camera_itr != feature_channel_itr->cameras.end();
           feature_camera_itr++) {
        // set up the title
        auto title = features_itr->name + "/" + feature_channel_itr->name +
                     "/" + feature_camera_itr->name + suffix;
        auto &display_image = display_map[title];

        // iterate through the keypoints and visualize
        for (uint32_t idx = 0; idx < feature_camera_itr->keypoints.size();
             ++idx) {
          const auto &keypoint = feature_camera_itr->keypoints[idx];
          const auto &keypoint_info = feature_camera_itr->feat_infos[idx];
          const auto &point = landmark_channel_itr->points.col(idx);
          int blue = 0;
          int red = std::max(0, 255 - 20 * (int)point(2));
          int green = std::min(255, 20 * (int)point(2));
          cv::Scalar kpColor(blue, green, red);
          cv::circle(display_image, keypoint.pt,
                     4.0 / std::sqrt(keypoint_info.precision), kpColor);
        }  // end for keypoints

        // print the number of raw features
        std::ostringstream tostr;
        tostr << feature_camera_itr->keypoints.size();
        cv::putText(display_image,
                    std::string("Features: " + tostr.str()).c_str(),
                    cv::Point(25, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);

        auto title2 = features_itr->name + "/" + feature_channel_itr->name +
                      "/" + feature_camera_itr->name + suffix;

        // show the images
        {
          std::lock_guard<std::mutex> lock(vis_mtx);
          cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
          cv::imshow(title2, display_image);
        }

      }  // end for camera
    }    // end for channel
  }      // end for rig
  {
    std::lock_guard<std::mutex> lock(vis_mtx);
    cv::waitKey(1);
  }
}

void showMatches(std::mutex &vis_mtx, CameraQueryCache &qdata,
                 std::vector<vtr::vision::RigMatches> &matches,
                 std::string suffix, bool plot_prediction) {
  // check if the required data is in the cache
  if (!qdata.rig_images.is_valid() || !qdata.rig_features.is_valid() ||
      !qdata.candidate_landmarks.is_valid() ||
      !qdata.map_landmarks.is_valid()) {
    return;
  }

  // make a track color for valid grayscale
  cv::Scalar validTrackColor(203, 201, 40);

  // make a track color for valid non-grayscale
  cv::Scalar ccValidTrackColor(0, 255, 0);

  // make an alternative track color for invalid tracks
  cv::Scalar invalidTrackColor(40, 201, 203);

  // make the prediction track color
  cv::Scalar predictionTrackColor(201, 40, 203);

  // get a map of images to titles
  auto display_map = setupDisplayImages(qdata, suffix);

  std::vector<RigFeatures> &features = *qdata.rig_features;
  std::vector<RigLandmarks> &candidate_landmarks = *qdata.candidate_landmarks;
  std::vector<LandmarkFrame> &map_landmarkframe = *qdata.map_landmarks;
  std::list<RigCalibration> &calibrations = *qdata.rig_calibrations;
  std::vector<RigFeatures>::iterator features_itr = features.begin();
  std::vector<RigMatches>::iterator matches_itr = matches.begin();
  std::vector<RigLandmarks>::iterator candidate_landmark_itr =
      candidate_landmarks.begin();
  std::vector<LandmarkFrame>::iterator map_landmark_itr =
      map_landmarkframe.begin();
  std::list<RigCalibration>::iterator calibration_itr = calibrations.begin();

  // containers for the feature prediction visualisation
  Eigen::Matrix<double, 3, 4> T;
  Eigen::Matrix<double, 3, 4> Ti;

  // iterate through each rig (for landmarks, matches and features)
  for (; features_itr != features.end() && matches_itr != matches.end() &&
         candidate_landmark_itr != candidate_landmarks.end() &&
         map_landmark_itr != map_landmarkframe.end() &&
         calibration_itr != calibrations.end();
       ++features_itr, ++matches_itr, ++candidate_landmark_itr,
       calibration_itr++) {
    // set up data for feature prediction
    vtr::vision::CameraIntrinsic &K = calibration_itr->intrinsics.at(0);

    // get the candidate transform given by a different function and transform
    // it to the camera frame
    if (qdata.T_r_m_prior.is_valid() == true) {
      lgmath::se3::Transformation T_q_m =
          (*qdata.T_sensor_vehicle) * (*qdata.T_r_m_prior) *
          ((*qdata.T_sensor_vehicle_map)[*qdata.live_id].inverse());

      // pre-cache the transform matrix
      T = K * T_q_m.matrix().topLeftCorner(3, 4);

      // pre-cache the inverse transform matrix
      Ti = K * T_q_m.matrix().inverse().topLeftCorner(3, 4);
    }

    // iterate through each channel
    std::vector<vtr::vision::ChannelFeatures>::iterator feature_channel_itr =
        features_itr->channels.begin();
    std::vector<vtr::vision::ChannelMatches>::iterator match_channel_itr =
        matches_itr->channels.begin();
    std::vector<vtr::vision::ChannelLandmarks>::iterator
        candidate_landmark_channel_itr =
            candidate_landmark_itr->channels.begin();
    std::vector<vtr::vision::ChannelObservations>::iterator
        map_obs_channel_itr = map_landmark_itr->observations.channels.begin();
    std::vector<vtr::vision::ChannelLandmarks>::iterator map_lm_channel_itr =
        map_landmark_itr->landmarks.channels.begin();

    for (;
         feature_channel_itr != features_itr->channels.end() &&
         match_channel_itr != matches_itr->channels.end() &&
         candidate_landmark_channel_itr !=
             candidate_landmark_itr->channels.end() &&
         map_obs_channel_itr != map_landmark_itr->observations.channels.end() &&
         map_lm_channel_itr != map_landmark_itr->landmarks.channels.end();
         ++feature_channel_itr, ++match_channel_itr,
         ++candidate_landmark_channel_itr, ++map_obs_channel_itr,
         map_lm_channel_itr++) {

      if (feature_channel_itr->cameras.empty() || 
         (feature_channel_itr->name == "RGB")) {
        continue;
      }

      // monocular scheme?
      bool monocular = feature_channel_itr->cameras.size() == 1 ? true : false;

      // get the first image to display matches
      std::vector<vtr::vision::Features>::iterator feature_camera_itr =
          feature_channel_itr->cameras.begin();

      // get the first image to display matches
      std::vector<vtr::vision::Observations>::iterator obs_camera_itr =
          map_obs_channel_itr->cameras.begin();

      // set up the title for this channel
      auto title = features_itr->name + "/" + feature_channel_itr->name + "/" +
                   feature_camera_itr->name + suffix;

      // get the display image
      auto &display_image = display_map[title];

      // iterate through each match
      for (vtr::vision::SimpleMatches::iterator match_itr =
               match_channel_itr->matches.begin();
           match_itr != match_channel_itr->matches.end(); match_itr++) {
        const auto &keypoint = feature_camera_itr->keypoints[match_itr->second];
        const auto &keypoint_info =
            feature_camera_itr->feat_infos[match_itr->second];

        const auto &mapkeypoint = obs_camera_itr->points[match_itr->first];

        bool valid = false;
        Eigen::Vector3d querypoint3d;
        if (candidate_landmark_channel_itr->valid.at(match_itr->second)) {
          valid = true;
          querypoint3d =
              candidate_landmark_channel_itr->points.col(match_itr->second);
        } else if (map_lm_channel_itr->valid.at(match_itr->first)) {
          valid = true;
          querypoint3d = map_lm_channel_itr->points.col(match_itr->first);
        }

        if (valid == true && feature_channel_itr->name == "grayscale") {
          cv::line(display_image, mapkeypoint, keypoint.pt, validTrackColor, 2);
        } else if (valid == true && feature_channel_itr->name != "grayscale") {
          cv::line(display_image, mapkeypoint, keypoint.pt, ccValidTrackColor,
                   2);
        } else {
          cv::line(display_image, mapkeypoint, keypoint.pt, invalidTrackColor,
                   2);
        }

        // adjust the feature colour by the depth
        int blue = 0;
        int red = std::max(0, 255 - 20 * (int)querypoint3d(2));
        int green = std::min(255, 20 * (int)querypoint3d(2));

        // generate the feature ellipse in the image
        cv::Scalar kpColor(blue, green, red);
        // extract eigenvectors/values for the covariance.
        Eigen::EigenSolver<Eigen::Matrix2d> eigen_solver;
        eigen_solver.compute(keypoint_info.covariance, true);
        // get the indexes of the maximal/minim eigenvalue
        Eigen::Matrix<std::complex<double>, 2, 1>::Index maxRow, maxCol, minRow,
            minCol;
        Eigen::Matrix<std::complex<double>, 2, 1> eigenvalues =
            eigen_solver.eigenvalues();
        eigenvalues.real().col(0).maxCoeff(&maxRow, &maxCol);
        eigenvalues.real().col(0).minCoeff(&minRow, &minCol);
        auto maxEigenvector = eigen_solver.eigenvectors().col(maxRow);
        // generate the ellipse from the major/minor axes
        double ex = maxEigenvector[0].real();
        double ey = maxEigenvector[1].real();
        double emajor = std::sqrt(eigenvalues[maxRow].real());
        double eminor = std::sqrt(eigenvalues[minRow].real());
        double angle = std::atan2(ey, ex) * 180.0 / M_PI;
        // place the ellipse in the image
        cv::ellipse(display_image, keypoint.pt, cv::Size(emajor, eminor), angle,
                    0.0, 360.0, kpColor);

        // visualise the feature prediction using T_q_m for monocular (but only
        // 20% or so)
        if (plot_prediction && monocular && valid &&
            qdata.T_r_m_prior.is_valid() &&  // *qdata.map_status != MAP_NEW &&
            !(match_itr->first % 20)) {
          vtr::vision::Point p_pred_map_pt;
          // transform the homogenised point from the map to the query frame
          Eigen::Vector3d p_map_pt_3d =
              map_lm_channel_itr->points.col(match_itr->first);
          Eigen::Vector3d map_mod_pt = T * p_map_pt_3d.homogeneous();

          // copy the new normalised pixel position
          p_pred_map_pt.x = map_mod_pt.hnormalized()(0);
          p_pred_map_pt.y = map_mod_pt.hnormalized()(1);

          cv::line(display_image, mapkeypoint, p_pred_map_pt,
                   predictionTrackColor, 2);
          // scale it by the desired window size
          float window_size_x = 5;
          float window_size_y = 5;
          cv::ellipse(display_image, p_pred_map_pt,
                      cv::Size(window_size_x, window_size_y), 0.0, 0.0, 360.0,
                      predictionTrackColor);
        }

        // visualise the feature prediction using T_q_m for stereo (but only 20%
        // or so)
        if (plot_prediction && !monocular && valid &&
            qdata.T_r_m_prior.is_valid() && !(match_itr->first % 20)) {
          vtr::vision::Point p_pred_query_pt;
          // transform the homogenised point from the map to the query frame
          Eigen::Vector3d qry_mod_pt = Ti * querypoint3d.homogeneous();

          // copy the new normalised pixel position
          p_pred_query_pt.x = qry_mod_pt.hnormalized()(0);
          p_pred_query_pt.y = qry_mod_pt.hnormalized()(1);

          cv::line(display_image, keypoint.pt, p_pred_query_pt,
                   predictionTrackColor, 2);
          // scale it by the desired window size
          float window_size_x = 5;
          float window_size_y = 5;
          cv::ellipse(display_image, p_pred_query_pt,
                      cv::Size(window_size_x, window_size_y), 0.0, 0.0, 360.0,
                      predictionTrackColor);
        }
#if false
        // visualise the feature prediction using H_q_m (but only 10% or so)
        if (plot_prediction && qdata.H_q_m_prior.is_valid() &&
            *qdata.map_status == MAP_NEW && !(match_itr->first % 10)) {
          // get the prior homography matrix
          Eigen::Matrix3d H = *qdata.H_q_m_prior;

          vtr::vision::Point p_pred_map_pt;

          Eigen::Vector3d map_pt_h(mapkeypoint.x, mapkeypoint.y, 1.0);
          // use the infinite homography to predict the location of the point
          Eigen::Vector3d map_mod_pt = H * map_pt_h;

          // copy the new normalised pixel position
          p_pred_map_pt.x = map_mod_pt.hnormalized()(0);
          p_pred_map_pt.y = map_mod_pt.hnormalized()(1);
          cv::line(display_image, mapkeypoint, p_pred_map_pt,
                   predictionTrackColor, 2);
          // scale it by the desired window size
          // float window_size_x = 5;
          // float window_size_y = 5;
          // cv::ellipse(display_image, p_pred_map_pt,
          // cv::Size(window_size_x,window_size_y),0.0,0.0,360.0,predictionTrackColor);
        }
#endif
      }  // end for matches

      // print the number of matches
      std::ostringstream tostr;
      tostr << match_channel_itr->matches.size();
      cv::putText(display_image, std::string("Matches: " + tostr.str()).c_str(),
                  cv::Point(25, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1);

      // show the images
      {
        std::lock_guard<std::mutex> lock(vis_mtx);
        cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::imshow(title, display_image);
      }
    }  // end for channel
  }    // end for rig
  {
    std::lock_guard<std::mutex> lock(vis_mtx);
    cv::waitKey(1);
  }
}

cv::Scalar getExperienceColor(int expID, int privID) {
  int blue = 0;
  int green = 0;
  int red = 0;
  auto color = (expID - privID) % 6;

  if (color == 0) {  // green
    blue = 78;
    green = 200;
    red = 67;
  } else if (color == 1) {  // purple
    blue = 110;
    green = 19;
    red = 94;
  } else if (color == 2) {  // orange
    blue = 37;
    green = 103;
    red = 238;
  } else if (color == 3) {  // blue
    blue = 122;
    green = 102;
    red = 37;
  } else if (color == 4) {  // gold
    blue = 11;
    green = 163;
    red = 251;
  } else if (color == 5) {  // grey
    blue = 50;
    green = 50;
    red = 50;
  }
  return cv::Scalar(blue, green, red, 125);
}

cv::Scalar getChannelColor(std::string channel_name) {
  int blue = 0;
  int green = 0;
  int red = 0;
  
  if (channel_name == "RGB") {  // green
    blue = 78;
    green = 200;
    red = 67;
  } else if (channel_name == "grayscale") {  // purple
    blue = 110;
    green = 19;
    red = 94;
  } else {  // orange
    blue = 37;
    green = 103;
    red = 238;
  } 

  return cv::Scalar(blue, green, red, 125);
}

void showMelMatches(std::mutex &vis_mtx, CameraQueryCache &qdata,
                    const pose_graph::RCGraph::ConstPtr &graph,
                    std::string suffix, int idx) {
  (void)idx;
  // check if the required data is in the cache
  if (!qdata.rig_names.is_valid() || !qdata.map_landmarks.is_valid() ||
      !qdata.ransac_matches.is_valid() ||
      !qdata.migrated_landmark_ids.is_valid()) {
    return;
  }

  // Inputs:
  auto &rig_names = *qdata.rig_names;
  auto &query_landmarks = *qdata.map_landmarks;
  auto &matches = *qdata.ransac_matches;
  auto &migrated_landmark_ids = *qdata.migrated_landmark_ids;

  // Project the map points in the query camera frame.
  auto &projected_map_points = *qdata.projected_map_points;

  // we want to display the left, grayscale map image
  std::string title =
      query_landmarks[0].observations.name + "/" +
      query_landmarks[0].observations.channels.back().name + "/" +
      query_landmarks[0].observations.channels.back().cameras[0].name + suffix;

  // setup the visualization image stream
  std::string stream_name = rig_names.at(0) + "_visualization_images";
  for (const auto &r : graph->runs())
    r.second->registerVertexStream<vtr_messages::msg::Image>(
        stream_name, true, pose_graph::RegisterMode::Existing);

  // get the map image from the graph.
  auto map_vertex = graph->at(*qdata.map_id);
  common::timing::SimpleTimer viz_timer;
  map_vertex->load(stream_name);
  auto ros_image =
      map_vertex->retrieveKeyframeData<vtr_messages::msg::Image>(stream_name);
  if (ros_image == nullptr) {
    LOG(WARNING)
        << "Could not retrieve visualization image from the graph! NOT "
           "displaying MEL matches.";
    return;
  }
  if (viz_timer.elapsedMs() >= 20) {
    LOG(WARNING) << __func__ << " loading an image took " << viz_timer;
  }
  auto input_image = messages::wrapImage(*ros_image);
  auto display_image = setupDisplayImage(input_image);

  for (unsigned rig_idx = 0; rig_idx < query_landmarks.size(); ++rig_idx) {
    const auto &query_rig_obs = query_landmarks[rig_idx].observations;
    const auto &query_rig_lm = query_landmarks[rig_idx].landmarks;
    for (unsigned channel_idx = 0; channel_idx < query_rig_obs.channels.size();
         ++channel_idx) {
      const auto &query_channel_obs = query_rig_obs.channels[channel_idx];
      const auto &query_channel_lm = query_rig_lm.channels[channel_idx];
      const auto &channel_matches = matches[rig_idx].channels[channel_idx];
      if (query_channel_obs.cameras.empty()) {
        continue;
      }
      auto &query_camera_obs = query_channel_obs.cameras[0];
      std::string channel_name = query_channel_obs.name;
      // set up the title for this channel
      // get the display image
      for (auto &match : channel_matches.matches) {
        // get the run id of the map landmark.
        auto map_lm_id = migrated_landmark_ids[match.first];
        auto from_vid = graph->fromPersistent(map_lm_id.from_id.persistent);
        uint32_t map_run = from_vid.majorId();
        const auto &keypoint = query_camera_obs.points[match.second];
        const auto &point = query_channel_lm.points.col(match.second);
        const auto &precision = query_camera_obs.precisions[match.second];

        const auto &map_keypoint = projected_map_points.col(match.first);
        cv::Point map_kp_cv(map_keypoint(0), map_keypoint(1));
        int blue = 0;
        int red = std::max(0, 255 - 20 * (int)point(2));
        int green = std::min(255, 20 * (int)point(2));
        cv::Scalar kpColor(blue, green, red);
        // cv::Scalar trackColor = getChannelColor(channel_name);

        cv::Scalar trackColor = getExperienceColor(
            map_run, map_vertex->id().majorId());  //(blue,green,red);

        cv::line(display_image, map_kp_cv, keypoint, trackColor, 3);
        if (channel_idx == 2) {
          cv::line(display_image, map_kp_cv, keypoint,
                   cv::Scalar(255, 255, 255), 1.5);
        }
        cv::circle(display_image, keypoint, 4.0 / std::sqrt(precision),
                   trackColor, 3);
        if (channel_idx == 2) {
          cv::circle(display_image, keypoint, 4.0 / std::sqrt(precision),
                     cv::Scalar(255, 255, 255), 1.5);
        }
      }
      tactic::EdgeTransform T_q_m;
      if (*qdata.success == true) {
        T_q_m = *qdata.T_r_m;
      } else {
        T_q_m = *qdata.T_r_m_prior;
      }
      // print the number of matches
      std::stringstream display_text;
      display_text.precision(3);
      display_text << "(" << sqrt(T_q_m.cov()(0, 0)) * 100 << "cm,"
                   << sqrt(T_q_m.cov()(1, 1)) * 100 << "cm,"
                   << sqrt(T_q_m.cov()(5, 5)) * 57.29577 << "deg)";
      cv::putText(display_image, display_text.str().c_str(), cv::Point(25, 370),
                  cv::FONT_HERSHEY_TRIPLEX, 1.0, cv::Scalar(255, 255, 255, 125),
                  3);

      // show the images
      {
        std::lock_guard<std::mutex> lock(vis_mtx);
        cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::imshow(title, display_image);
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(vis_mtx);
    cv::waitKey(1);
  }
}

}  // namespace visualize
}  // namespace vision
}  // namespace vtr
