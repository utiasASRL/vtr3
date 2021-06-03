#pragma once

#include <cstdint>

namespace vtr {
namespace pose_graph {

/**
 * \brief Window type used for picking start/stop time when reindexing streams
 */
enum class WindowType : uint8_t { Before, Center, After };

/** \brief Mode for registering a vertex stream */
enum class RegisterMode : uint8_t {
  Create,    // No data exists; create a read/write stream
  Existing,  // Data exists but no stream; create a read-only stream
};

}  // namespace pose_graph
}  // namespace vtr