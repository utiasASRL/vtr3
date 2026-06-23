// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file image_error_predictor_network.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */

#ifndef VTR_PATH_PLANNING_ERROR_PREDICTORS_IMAGE_ERROR_PREDICTOR_NETWORK_HPP
#define VTR_PATH_PLANNING_ERROR_PREDICTORS_IMAGE_ERROR_PREDICTOR_NETWORK_HPP

#include "vtr_path_planning/error_predictors/base_error_predictor.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <lgmath/se3/Transformation.hpp>

namespace vtr {
namespace path_planning {

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

class ImageErrorPredictorNetwork : public BaseErrorPredictor {
public:
  PTR_TYPEDEFS(ImageErrorPredictorNetwork);

  ImageErrorPredictorNetwork(std::string model_path, bool use_gpu = false);
  ~ImageErrorPredictorNetwork();

  void predictError(const RobotState& robot_state, const tactic::Timestamp& curr_time) override {}
  void predictError(const RobotState& robot_state, const tactic::Timestamp& curr_time,
                    std::vector<lgmath::se3::Transformation>& reference_poses);
  void setInputs(const std::shared_ptr<ImageMsg>& sig_img_msg,
                 const std::shared_ptr<ImageMsg>& dpt_img_msg,
                 const std::shared_ptr<ImuMsg>& imu_msg);
  void clearInputs();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  std::shared_ptr<ImageMsg> cur_sig_img_msg_ = nullptr;
  std::shared_ptr<ImageMsg> cur_dpt_img_msg_ = nullptr;
  std::shared_ptr<ImuMsg> cur_imu_msg_ = nullptr;
};

}  // namespace path_planning
}  // namespace vtr

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_IMAGE_ERROR_PREDICTOR_NETWORK_HPP
