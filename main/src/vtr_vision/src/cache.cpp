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
 * \file cache.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_tactic/utils/cache_container.inl>
#include <vtr_vision/cache.hpp>

namespace vtr {
namespace common {
template class cache_ptr<std::mutex>;
template class cache_ptr<std::map<tactic::VertexId, tactic::EdgeTransform>>;
template class cache_ptr<std::list<vision::RigImages>>;
template class cache_ptr<std::list<vision::RigCalibration>>;
template class cache_ptr<std::vector<vision::RigFeatures>>;
template class cache_ptr<std::vector<vision::RigLandmarks>>;
template class cache_ptr<std::vector<vision::LandmarkFrame>>;
template class cache_ptr<std::vector<vision::RigMatches>>;
template class cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>>;
template class cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>>;
template class cache_ptr<std::vector<vtr_messages::msg::Match>>;
template class cache_ptr<Eigen::Matrix3Xd>;
template class cache_ptr<Eigen::Matrix4Xd>;
template class cache_ptr<tactic::RunIdSet>;
template class cache_ptr<steam::se3::SteamTrajInterface>;
template class cache_ptr<std::unordered_map<
    int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>>;
template class cache_ptr<vision::LandmarkMap>;
template class cache_ptr<vision::SteamPoseMap>;
template class cache_ptr<pose_graph::RCGraphBase::Ptr>;
template class cache_ptr<vtr_messages::msg::LocalizationStatus>;
template class cache_ptr<vision::MigrationMap>;
template class cache_ptr<common::timing::SimpleTimer>;
}  // namespace common
}  // namespace vtr