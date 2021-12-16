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
 * \brief GraphmemManagerModule class methods definition
 *
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

void GraphMemManagerModule::runImpl(QueryCache &qdata, OutputCache &,
                                    const Graph::Ptr &,
                                    const TaskExecutor::Ptr &executor) {
  if (!qdata.map_id->isValid() || !qdata.live_id->isValid()) return;
  if (*qdata.map_id == last_map_id_) return;

  // input to the graph memory manager async task.
  qdata.graph_mem_async.emplace(*qdata.live_id, *qdata.map_id);

  executor->dispatch(
      std::make_shared<Task>(shared_from_this(), qdata.shared_from_this()));
}

void GraphMemManagerModule::runAsyncImpl(QueryCache &qdata, OutputCache &,
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

  const auto live_id = qdata.graph_mem_async->first;
  const auto map_id = qdata.graph_mem_async->second;

  auto eval = std::make_shared<PrivilegedEvaluator<Graph>>();
  eval->setGraph((void *)graph.get());

  std::vector<Graph::VertexPtr> vertices;
  vertices.reserve(2 * config_->window_size);

  // subgraph is necessary to avoid concurrency issues.
  const auto subgraph = graph->getSubgraph(map_id, config_->window_size, eval);
  auto iter = subgraph->beginDfs(map_id, config_->window_size, eval);
  for (; iter != subgraph->end(); ++iter) vertices.push_back(iter->v());

  for (auto &&vertex : vertices) {
    // load up the vertex and its spatial neighbors.
    vid_life_map_[vertex->id()] = config_->vertex_life_span;
    for (auto &&spatial_vid : vertex->spatialNeighbours()) {
      if (spatial_vid.majorId() != live_id.majorId())
        vid_life_map_[spatial_vid] = config_->vertex_life_span;
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