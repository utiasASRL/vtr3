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
 * \file navigator.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_navigation_v2/navigator.hpp"

#include "vtr_common/utils/filesystem.hpp"

namespace vtr {
namespace navigation {

Navigator::Navigator(const rclcpp::Node::SharedPtr& node) : node_(node) {
  el::Helpers::setThreadName("navigator");

  CLOG(INFO, "navigator") << "Starting VT&R3 system - hello!";

  /// data storage directory (must have been set at this moment)
  auto data_dir = node_->get_parameter("data_dir").get_value<std::string>();
  data_dir = common::utils::expand_user(common::utils::expand_env(data_dir));
  CLOG(INFO, "navigator") << "Data directory set to: " << data_dir;

  /// graph map server (pose graph callback, tactic callback)
  graph_map_server_ = std::make_shared<GraphMapServer>();

  /// pose graph \todo set up callback
  auto new_graph = node->declare_parameter<bool>("start_new_graph", false);
  graph_ = tactic::Graph::MakeShared(data_dir + "/graph", !new_graph);

  CLOG_IF(!graph_->numberOfVertices(), INFO, "navigator")
      << "Creating a new pose graph.";
  CLOG_IF(graph_->numberOfVertices(), INFO, "navigator")
      << "Loaded pose graph has " << graph_->numberOfVertices() << " vertices.";

  ///
  graph_map_server_->start(node, graph_);

  CLOG(INFO, "navigator") << "VT&R3 initialization done!";
}

Navigator::~Navigator() {
  graph_.reset();

  graph_map_server_.reset();

  CLOG(INFO, "navigator") << "VT&R3 destruction done! Bye-bye.";
}

}  // namespace navigation
}  // namespace vtr
