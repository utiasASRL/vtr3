#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include <vtr_logging/logging.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

/**
 * \brief Memory manager loads a window of the map into memory based on desired
 * streams, centered around trunk vertex in the localization chain. Unloads
 * vertices outside of the window.
 */
class BaseMemoryManager {
 public:
  struct Config {
    /** \brief Enable/Disable this memory manager */
    bool enable;
  };

  BaseMemoryManager(const Config& config) : config_(config) {}
  ~BaseMemoryManager() { stop(); }

  /** \brief starts the loading thread. */
  void start();

  /** \brief stops the loading thread. */
  void stop();

 protected:
  virtual bool checkUpdate() = 0;
  virtual void manageMemory() = 0;

 private:
  /**
   * \brief the main loading function.
   * On a loop until the loading thread is stopped, this function inspects the
   * localization chain for a change in the trunk vertex ID. Upon a trunk vertex
   * ID change, this thread loads a window of vertices into memory and unloads
   * vertices outside of the window.
   */
  void run();

  Config config_;

  /** \brief The management thread. */
  std::thread management_thread_;

  /** \brief The flag to kill the management thread. */
  std::atomic<bool> kill_thread_ = false;
};

}  // namespace tactic
}  // namespace vtr
