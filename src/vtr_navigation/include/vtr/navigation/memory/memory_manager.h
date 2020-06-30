#pragma once

#include <atomic>
#include <thread>

#include <vtr/navigation/types.h>

#include <asrl/pose_graph/path/LocalizationChain.hpp>

namespace vtr {
namespace navigation {
/** \brief MemoryManager Loads a window of the map into memory based on desired
 * streams, centered around trunk vertex in the localization chain. Unloads
 * vertices outside of the window.
 */
class MemoryManager {
 public:
  MemoryManager() : kill_thread_(false) {}
  ~MemoryManager() { stop(); }

  /** \brief starts the loading thread.
   */
  void start();

  /** \brief stops the loading thread.
   */
  void stop();

 protected:
  virtual bool checkUpdate() = 0;
  virtual void manageMemory() = 0;

 private:
  /** \brief the main loading function.
   *
   * On a loop until the loading thread is stopped, this function inspects the
   * localization chain for a change in the trunk vertex ID. Upon a trunk vertex
   * ID change, this thread loads a window of vertices into memory and unloads
   * vertices outside of the window.
   */
  void run();

  /** \brief The management thread.
   */
  std::thread management_thread_;

  /** \brief The flag to kill the management thread.
   */
  std::atomic<bool> kill_thread_;
};

}  // namespace navigation
}  // namespace vtr
