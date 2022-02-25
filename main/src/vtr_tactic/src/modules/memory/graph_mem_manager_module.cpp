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
 * \file graph_mem_manager_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/modules/memory/graph_mem_manager_module.hpp"

namespace vtr {
namespace tactic {

auto GraphMemManagerModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->vertex_life_span = node->declare_parameter<int>(param_prefix + ".vertex_life_span", config->vertex_life_span);
  config->window_size = node->declare_parameter<int>(param_prefix + ".window_size", config->window_size);
  // clang-format on
  return config;
}

void GraphMemManagerModule::run_(QueryCache &qdata, OutputCache &,
                                 const Graph::Ptr &,
                                 const TaskExecutor::Ptr &executor) {
  if (!qdata.vid_loc->isValid() || !qdata.vid_odo->isValid()) return;
  if (*qdata.vid_loc == last_vid_) return;

  // input to the graph memory manager async task.
  qdata.graph_mem_async.emplace(*qdata.vid_odo, *qdata.vid_loc);

  executor->dispatch(std::make_shared<Task>(
      shared_from_this(), qdata.shared_from_this(), 0, Task::DepIdSet{},
      Task::DepId{}, "Graph Mem Manager", *qdata.vid_odo));
}

void GraphMemManagerModule::runAsync_(QueryCache &qdata, OutputCache &,
                                      const Graph::Ptr &graph,
                                      const TaskExecutor::Ptr &,
                                      const Task::Priority &,
                                      const Task::DepId &) {
  std::lock_guard<std::mutex> lock(vid_life_map_mutex_);

  if (!qdata.graph_mem_async.valid()) {
    std::string err{"qdata.graph_mem_async not set"};
    CLOG(ERROR, "tactic.module.graph_mem_manager") << err;
    throw std::runtime_error(err);
  }

  const auto vid_odo = qdata.graph_mem_async->first;
  const auto vid_loc = qdata.graph_mem_async->second;

  auto eval = std::make_shared<PrivilegedEvaluator<Graph>>(*graph);

  std::vector<VertexId> vertices;
  vertices.reserve(2 * config_->window_size);

  // subgraph is necessary to avoid concurrency issues.
  const auto subgraph = graph->getSubgraph(vid_loc, config_->window_size, eval);
  auto iter = subgraph->beginDfs(vid_loc, config_->window_size, eval);
  for (; iter != subgraph->end(); ++iter) vertices.push_back(iter->v()->id());

  for (auto &&vertex : vertices) {
    // load up the vertex and its spatial neighbors.
    vid_life_map_[vertex] = config_->vertex_life_span;
    for (auto &&vid : graph->neighbors(vertex)) {
      if (graph->at(EdgeId(vid, vertex))->isSpatial() &&
          (vid.majorId() != vid_odo.majorId()))
        vid_life_map_[vid] = config_->vertex_life_span;
    }
  }

  // take life from all vertices.
  CLOG(DEBUG, "tactic.module.graph_mem_manager")
      << "Current life map: " << vid_life_map_;
  std::vector<VertexId> to_die;
  for (auto iter = vid_life_map_.begin(); iter != vid_life_map_.end(); ++iter) {
    iter->second--;
    // if the vertices life is zero, then sentence it to die.
    if (iter->second <= 0) {
      auto vertex = graph->at(iter->first);
      vertex->unload();
      to_die.push_back(iter->first);
    }
  }

  // kill vertices that are scheduled to die.
  for (const auto &death : to_die) vid_life_map_.erase(death);
}

}  // namespace tactic
}  // namespace vtr