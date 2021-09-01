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
 * \brief Template instantiation of types used in QueryCache
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/utils/cache_container.inl>

namespace vtr {
namespace common {

template class cache_ptr<int>;
template class cache_ptr<float>;
template class cache_ptr<bool>;
template class cache_ptr<bool, true>;
template class cache_ptr<std::string>;
template class cache_ptr<std::vector<std::string>>;
template class cache_ptr<std::vector<bool>>;
template class cache_ptr<std::vector<float>>;
template class cache_ptr<std::vector<double>>;

template class cache_ptr<rclcpp::Node>;
template class cache_ptr<rclcpp::Time>;
template class cache_ptr<vtr_messages::msg::TimeStamp>;
template class cache_ptr<tactic::PipelineMode>;
template class cache_ptr<tactic::VertexId>;
template class cache_ptr<tactic::LocalizationChain>;
template class cache_ptr<lgmath::se3::TransformationWithCovariance>;
template class cache_ptr<tactic::KeyframeTestResult>;
template class cache_ptr<steam::se3::SteamTrajInterface>;

}  // namespace common
}  // namespace vtr