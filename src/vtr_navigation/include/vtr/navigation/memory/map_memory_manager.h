#pragma once

#include <atomic>
#include <thread>

#include <vtr/navigation/memory/memory_config.h>
#include <vtr/navigation/memory/memory_manager.h>
#include <vtr/navigation/types.h>

#include <asrl/pose_graph/path/LocalizationChain.hpp>

namespace vtr {
namespace navigation {

/** \brief Loads a window of the map into memory based on desired streams,
 * centered around trunk vertex in the localization chain. Unloads vertices
 * outside of the window.
 */
class MapMemoryManager : public MemoryManager {
 public:
  /** \brief constructor
   *
   * \param graph A const pointer to a map graph.
   * \param chain A const reference to a localization chain.
   * \param config Map memory manager config.
   */
  MapMemoryManager(const std::shared_ptr<Graph> graph,
                   const asrl::pose_graph::LocalizationChain &chain,
                   MapMemoryManagerConfig &config);
  ~MapMemoryManager() {}

  /** \brief Gets the number of loaded vertices.
   *
   * \return the number of loaded vertices.
   */
  int numLoadedVertices() { return life_map_.size(); }

 protected:
  virtual bool checkUpdate();
  virtual void manageMemory();

 private:
  MapMemoryManagerConfig config_;

  /** \brief Loads the specified streams for the given vertex.
   *
   * \param vertex The vertex to load.
   * \param streams_to_load The vector of streams to load from the vertex.
   */
  void load(std::shared_ptr<asrl::pose_graph::RCVertex> &vertex,
            const std::vector<std::string> &streams_to_load);

  /** \brief unloads dead vertices.
   */
  void unloadDeadVertices();

  /** \brief loads vertices around
   */
  void loadVertices();

  /** \brief The currently loaded trunk ID.
   */
  VertexId loaded_trunk_id_;

  /** \brief Maps vertex ids to life spans.
   */
  std::unordered_map<VertexId, int> life_map_;

  /** \brief A constant reference to the localization chain.
   */
  const asrl::pose_graph::LocalizationChain &chain_;

  /** \brief A constant shared pointer to the pose graph.
   */
  const std::shared_ptr<Graph> graph_;

  /** \brief The management thread.
   */
  std::thread management_thread_;

  /** \brief The flag to kill the management thread.
   */
  std::atomic<bool> kill_thread_;
};

}  // namespace navigation
}  // namespace vtr
