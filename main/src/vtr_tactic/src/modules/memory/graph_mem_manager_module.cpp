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
#include <vtr_tactic/modules/memory/graph_mem_manager_module.hpp>

namespace vtr {
namespace tactic {

using namespace std::literals::chrono_literals;

void GraphMemManagerModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->vertex_life_span = node->declare_parameter<int>(param_prefix + ".vertex_life_span", config_->vertex_life_span);
  config_->window_size = node->declare_parameter<int>(param_prefix + ".window_size", config_->window_size);
  // clang-format on
}

void GraphMemManagerModule::runImpl(QueryCache &qdata,
                                    const Graph::ConstPtr &graph) {
  if (!task_queue_) return;
  if (!qdata.map_id->isValid() || !qdata.live_id->isValid()) return;
  if (*qdata.map_id == last_map_id_) return;

  const auto live_id = *qdata.live_id;
  const auto map_id = *qdata.map_id;

  task_queue_->dispatch(/* priority */ 9, [this, graph, live_id, map_id]() {
    std::lock_guard<std::mutex> lck(vid_life_map_mutex_);

    // do a search out on the chain, up to the lookahead distance.
    PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
    evaluator->setGraph((void *)graph.get());
    // search with the lock to get vertices
    std::vector<Graph::VertexPtr> vertices;
    vertices.reserve(2 * config_->window_size);
    graph->lock();
    auto itr = graph->beginDfs(map_id, config_->window_size, evaluator);
    for (; itr != graph->end(); ++itr) vertices.push_back(itr->v());

    for (auto &vertex : vertices) {
      // load up the vertex and its spatial neighbors.
      vid_life_map_[vertex->id()] = config_->vertex_life_span;
      for (auto &spatial_vid : vertex->spatialNeighbours()) {
        if (spatial_vid.majorId() != live_id.majorId())
          vid_life_map_[spatial_vid] = config_->vertex_life_span;
      }
    }
    graph->unlock();

    // take life from all vertices.
    auto life = vid_life_map_.begin();
    std::vector<decltype(life)> to_die;
    CLOG(DEBUG, "tactic.module.graph_mem_manager")
        << "Current life map: " << vid_life_map_;
    for (; life != vid_life_map_.end(); ++life) {
      life->second--;
      // if the vertices life is zero, then sentence it to die.
      if (life->second <= 0) {
        auto vertex = graph->at(life->first);
        vertex->unload();
        to_die.push_back(life);
      }
    }

    // kill vertices that are scheduled to die.
    for (auto &death : to_die) vid_life_map_.erase(death);
  });
}

}  // namespace tactic
}  // namespace vtr