#pragma once

#include <vtr/navigation/memory/memory_config.h>
#include <vtr/navigation/memory/memory_manager.h>
#include <vtr/navigation/tactics/basic_tactic.h>

namespace vtr {
namespace navigation {

/** \brief Loads a window of the map into memory based on
 * desired streams, centered around trunk vertex in the localization chain.
 * Unloads vertices outside of the window.
 */
class LiveMemoryManager : public MemoryManager {
 public:
  /** \brief constructor
   *
   * \param graph A const pointer to a map graph.
   * \param tactic A const reference to a localization chain.
   * \param config Live memory manager config.
   */
  LiveMemoryManager(const std::shared_ptr<Graph> graph, BasicTactic *tactic,
                    LiveMemoryManagerConfig &config);
  ~LiveMemoryManager() {}

 protected:
  virtual bool checkUpdate();
  virtual void manageMemory();

 private:
  VertexId live_vertex_;
  VertexId vertex_to_unload;

  std::shared_ptr<const Graph> graph_;

  BasicTactic *tactic_;

  LiveMemoryManagerConfig config_;
};

}  // namespace navigation
}  // namespace vtr
