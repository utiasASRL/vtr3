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
 * \file navigator.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "rclcpp/rclcpp.hpp"

#include "vtr_navigation_v2/graph_map_server.hpp"
#include "vtr_tactic/tactic_v2.hpp"

namespace vtr {
namespace navigation {

using namespace vtr::tactic;
using namespace vtr::pose_graph;
// using namespace vtr::mission_planning;

class Navigator {
 public:
  Navigator(const rclcpp::Node::SharedPtr& node);
  ~Navigator();

 private:
  /** \brief ROS-handle for communication */
  const rclcpp::Node::SharedPtr node_;

  /// VTR building blocks
  GraphMapServer::Ptr graph_map_server_;
  tactic::Graph::Ptr graph_;
};

}  // namespace navigation
}  // namespace vtr
