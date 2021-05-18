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

  /**
   * \brief constructor
   * \param graph A const pointer to a map graph.
   * \param tactic A const reference to a localization chain.
   * \param config Live memory manager config.
   */
  LiveMemoryManager(const Config &config,
                    mission_planning::StateMachineInterface *smi,
                    Graph::ConstPtr graph)
      : BaseMemoryManager(config),
        config_(config),
        tactic_(smi),
        graph_(graph) {}
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
