// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file types.hpp
 * \brief Header defining types used in VTR radar package
 * \details
 *
 * \author Sam Qiao, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <array>
#include <memory>
#include <vector>

// opencv definitions
#include <opencv2/core/core.hpp>


namespace vtr {
namespace radar{

struct RadarData {
 /// The radar azimuth timestamps (ns epoch time)
  std::vector<int64_t> azimuth_times;

  /// The radar azimuth angles
  std::vector<double> azimuth_angles;

  /// The OpenCV image for the FFT scan
  cv::Mat fft_scan;
  cv::Mat cartesian;

  // Chirp information
  std::vector<bool> up_chirps;
};

}  // namespace radar
}  // namespace vtr 
