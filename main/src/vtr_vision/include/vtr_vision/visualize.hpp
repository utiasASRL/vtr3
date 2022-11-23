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
 * \file visualize.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <opencv2/core/core.hpp>

#include <vtr_vision/cache.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {
namespace visualize {

/**
 * \brief Sets up a map of RGB images used for visualization, based on all of
 * the rig images.
 */
std::map<std::string, cv::Mat> setupDisplayImages(CameraQueryCache &qdata,
                                                  std::string suffix = "");

/** \brief */
void showStereoMatches(std::mutex &vis_mtx, CameraQueryCache &qdata,
                       std::string suffix = "");

/** \brief Adds visual features with depth coloring to the display images. */
void showRawFeatures(std::mutex &vis_mtx, CameraQueryCache &qdata,
                     std::string suffix = "");

/** \brief Adds visual features with depth coloring to the display images. */
void showFeatures(std::mutex &vis_mtx, CameraQueryCache &qdata,
                  std::string suffix = "");

/** \brief */
void showMelMatches(std::mutex &vis_mtx, CameraQueryCache &qdata,
                    const pose_graph::RCGraph::ConstPtr &graph,
                    std::string suffix = "", int img_idx = 0);

/** \brief Adds visual features with depth coloring to the display images. */
void showMatches(std::mutex &vis_mtx, CameraQueryCache &qdata,
                 std::vector<vision::RigMatches> &matches,
                 std::string suffix = "", bool plot_prediction = false);

/** \brief Adds disparity images for display. */
void showDisparity(std::mutex &vis_mtx, CameraQueryCache &qdata, 
                   std::string suffix = "");

}  // namespace visualize
}  // namespace vision
}  // namespace vtr
