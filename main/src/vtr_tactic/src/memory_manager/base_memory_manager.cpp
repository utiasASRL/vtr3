#include <vtr_tactic/memory_manager/base_memory_manager.hpp>

namespace vtr {
namespace tactic {

void BaseMemoryManager::start() {
  // stop the thread and start it again.
  // stop();
  kill_thread_ = false;
  if (config_.enable)
    management_thread_ = std::thread(&BaseMemoryManager::run, this);
}

void BaseMemoryManager::stop() {
  kill_thread_ = true;
  if (config_.enable) {
    LOG(INFO) << "Sending exit signal to memory manager run thread";
    management_thread_.join();
    LOG(INFO) << "Memory manager run thread finished";
  }
}

void BaseMemoryManager::run() {
  LOG(INFO) << "Starting the memory manager thread!";
  // while we are still alive
  while (!kill_thread_) {
    // if the chain is not empty, and the trunk has moved, then its time to
    // manage the map memory.
    if (checkUpdate()) {
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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace tactic
}  // namespace vtr
