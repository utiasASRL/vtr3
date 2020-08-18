#pragma once

#include <string>
#include <vector>

namespace vtr {
namespace navigation {

struct MemoryManagerConfig {
  /** \brief Enable/Disable this memory manager
   */
  bool enable;

  /** \brief The size of the window to load, centered around the trunk vertex.
   *
   * This is used by the map memory manager, but also examined by the live
   * memory manager.
   */
  int lookahead_distance;
};

struct LiveMemoryManagerConfig : MemoryManagerConfig {
  /** \brief How large a lookbackwindow from the live vertex to keep in memory
   */
  int window_size;
};

struct MapMemoryManagerConfig : MemoryManagerConfig {
  /** \brief The number of trunk changes to hold onto vertices that were not in
   * the window.
   */
  int vertex_life_span;

  /** \brief The collection of streams to load in all vertices.
   */
  std::vector<std::string> streams_to_load;

  /** \brief The collection of streams to load in privileged vertices only.
   */
  std::vector<std::string> priv_streams_to_load;
};

}  // namespace navigation
}  // namespace vtr
