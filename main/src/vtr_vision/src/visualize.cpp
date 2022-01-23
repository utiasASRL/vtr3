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
 * \author Autonomous Space Robotics Lab (ASRL)
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/visualize.hpp>

namespace vtr {
namespace vision {

cv::Mat setupDisplayImage(cv::Mat input_image) {
  // create a visualization image to draw on.
  cv::Mat display_image;
  if (input_image.type() == CV_8UC1) {
    cv::cvtColor(input_image, display_image, cv::COLOR_GRAY2RGB);
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

void showRawFeatures(std::mutex &vis_mtx, CameraQueryCache &qdata,
                     std::string suffix) {
  // check if the required data is in the cache
  if (!qdata.rig_images.valid() || !qdata.rig_features.valid()) return;

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

}  // namespace vision
}  // namespace vtr
