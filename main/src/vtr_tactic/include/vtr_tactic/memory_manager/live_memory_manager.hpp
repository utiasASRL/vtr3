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
 * \file live_memory_manager.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/memory_manager/base_memory_manager.hpp>

namespace vtr {
namespace tactic {

/**
 * \brief Loads a window of the map into memory based on
 * desired streams, centered around trunk vertex in the localization chain.
 * Unloads vertices outside of the window.
 */
class LiveMemoryManager : public BaseMemoryManager {
 public:
  struct Config : public BaseMemoryManager::Config {
    /** \brief Size of the window to load, centered around the trunk vertex. */
    unsigned lookahead_distance;
    /** \brief Size of the lookback window from live vertex to keep in memory */
    unsigned window_size;
  };

  /** \brief constructor */
  LiveMemoryManager(const Config &config,
                    mission_planning::StateMachineInterface *smi,
                    Graph::ConstPtr graph)
      : BaseMemoryManager(config),
        config_(config),
        graph_(graph),
        tactic_(smi) {}
  ~LiveMemoryManager() {}

 protected:
  virtual bool checkUpdate();
  virtual void manageMemory();

 private:
  VertexId live_vertex_ = VertexId::Invalid();
  VertexId vertex_to_unload_ = VertexId::Invalid();

  Config config_;

  Graph::ConstPtr graph_;

  mission_planning::StateMachineInterface *tactic_;
};

}  // namespace tactic
}  // namespace vtr
