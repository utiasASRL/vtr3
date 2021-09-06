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
 * \file graph_mem_manager_module.hpp
 * \brief GraphmemManagerModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/** \brief A tactic module template */
class GraphMemManagerModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "graph_mem_manager";

  /** \brief Collection of config parameters */
  struct Config {
    int vertex_life_span = 10;
    int window_size = 5;
  };

  GraphMemManagerModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(QueryCache &, const Graph::ConstPtr &) override;

  /** \brief mutex to protect access to life map */
  std::mutex vid_life_map_mutex_;

  /** \brief Maps vertex ids to life spans. */
  std::unordered_map<VertexId, int> vid_life_map_;

  VertexId last_map_id_ = VertexId::Invalid();

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;  /// \todo no need to be a shared pointer.
};

}  // namespace tactic
}  // namespace vtr