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
 * \file sensor_model_base.hpp
 * \brief Header file for the ASRL vision package
 * \details Several convenient typedefs
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>

namespace vtr {
namespace vision {

// Feature keypoints
typedef cv::KeyPoint Keypoint;
typedef std::vector<Keypoint> Keypoints;
typedef Eigen::Map<Eigen::Matrix<float, 2, Eigen::Dynamic>, Eigen::Unaligned,
                   Eigen::Stride<sizeof(Keypoint), sizeof(float)> >
    KeypointsMap;
typedef Eigen::Map<const Eigen::Matrix<float, 2, Eigen::Dynamic>,
                   Eigen::Unaligned,
                   Eigen::Stride<sizeof(Keypoint) / sizeof(float), 0> >
    KeypointsConstMap;

// Camera points
typedef Eigen::Vector2f CameraPoint;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic> CameraPoints;
typedef Eigen::Vector3f HomogeneousPoint;
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> HomogeneousPoints;
// To convert CameraPoints to HomogeneousPoints,
// homo = cam.colwise().homogeneous();
// To convert HomogeneousPoints to CameraPoints,
// cam = homo.hnormalized()

// Whitened measurement errors
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> ErrorList;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MeasVarList;

}  // namespace vision
}  // namespace vtr
