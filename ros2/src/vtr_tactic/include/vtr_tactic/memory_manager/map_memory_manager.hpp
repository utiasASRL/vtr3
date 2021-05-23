#pragma once

#include <vtr_tactic/memory_manager/base_memory_manager.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

/**
 * \brief Loads a window of the map into memory based on desired streams,
 * centered around trunk vertex in the localization chain. Unloads vertices
 * outside of the window.
 */
class MapMemoryManager : public BaseMemoryManager {
 public:
  struct Config : public BaseMemoryManager::Config {
    /** \brief Size of the window to load, centered around the trunk vertex. */
    int lookahead_distance;
    /**
     * \brief The number of trunk changes to hold onto vertices that were not
     * in the window.
     */
    int vertex_life_span;

    /** \brief The collection of streams to load in all vertices. */
    std::vector<std::string> streams_to_load;

    /** \brief The collection of streams to load in privileged vertices only. */
    std::vector<std::string> priv_streams_to_load;
  };

  /**
   * \brief constructor
   * \param graph A const pointer to a map graph.
   * \param chain A const reference to a localization chain.
   * \param config Map memory manager config.
   */
  MapMemoryManager(const Config &config,
                   const std::shared_ptr<std::mutex> &chain_mutex_ptr,
                   const LocalizationChain &chain, Graph::Ptr graph)
      : BaseMemoryManager(config),
        config_(config),
        chain_mutex_ptr_(chain_mutex_ptr),
        chain_(chain),
        graph_(graph) {}
  ~MapMemoryManager() {}

  /**
   * \brief Gets the number of loaded vertices.
   * \return the number of loaded vertices.
   */
  int numLoadedVertices() { return life_map_.size(); }

 protected:
  virtual bool checkUpdate();
  virtual void manageMemory();

 private:
  /**
   * \brief Loads the specified streams for the given vertex.
   * \param vertex The vertex to load.
   * \param streams_to_load The vector of streams to load from the vertex.
   */
  void load(Vertex::Ptr &vertex,
            const std::vector<std::string> &streams_to_load);

  /** \brief unloads dead vertices. */
  void unloadDeadVertices();

  /** \brief loads vertices around */
  void loadVertices();

  /** \brief The currently loaded trunk ID. */
  VertexId trunk_id_ = VertexId::Invalid();
  VertexId twig_id_ = VertexId::Invalid();

  /** \brief Maps vertex ids to life spans. */
  std::unordered_map<VertexId, int> life_map_;

  Config config_;

  const std::shared_ptr<std::mutex> chain_mutex_ptr_;
  /** \brief A constant reference to the localization chain. */
  const LocalizationChain &chain_;

  /** \brief A constant shared pointer to the pose graph. */
  Graph::Ptr graph_;

  /** \brief The management thread. */
  std::thread management_thread_;

  /** \brief The flag to kill the management thread. */
  std::atomic<bool> kill_thread_;
};

}  // namespace tactic
}  // namespace vtr
