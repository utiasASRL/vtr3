#include <chrono>

#include <vtr/navigation/memory/memory_manager.h>

namespace vtr {
namespace navigation {

void MemoryManager::start() {
  // stop the thread and start it again.
  // stop();
  kill_thread_ = false;
  management_thread_ = std::thread(&MemoryManager::run, this);
}

void MemoryManager::stop() {
  kill_thread_ = true;
  LOG(INFO) << __func__ << "Sending exit signal to run thread";
  management_thread_.join();
  LOG(INFO) << __func__ << " Thread finished";
}

void MemoryManager::run() {
  LOG(INFO) << "Starting map manger thread!!";
  // while we are still alive
  while (kill_thread_ == false) {
    // if the chain is not empty, and the trunk has moved, then its time to
    // manage the map memory.
    if (checkUpdate() == true) {
      try {
        manageMemory();
      } catch (std::runtime_error &e) {
        LOG(ERROR) << __func__ << " " << e.what();
      } catch (...) {
        LOG(ERROR) << __func__
                   << " caught an exception while managing memory!!";
      }
    }
    // if there is nothing to do, then sleep for a bit.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace navigation
}  // namespace vtr
