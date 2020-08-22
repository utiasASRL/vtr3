#pragma once
#if 0
#include <mutex>
#include <robochunk/base/ChunkSerializer.hpp>
#include <robochunk/base/DataBubble.hpp>
#endif
namespace vtr {
namespace pose_graph {
#if 0
struct RobochunkIO {
  typedef std::shared_ptr<robochunk::base::ChunkStream> StreamPtr;
  typedef std::shared_ptr<robochunk::base::ChunkSerializer> SerializerPtr;

  StreamPtr first;
  SerializerPtr second;
  std::recursive_mutex read_mtx;
  std::recursive_mutex write_mtx;

  typedef std::unique_lock<std::recursive_mutex> Guard;
  struct RWGuard{ Guard read, write; };
  RWGuard lock(bool read = true, bool write = true) {
    RWGuard rwg {{read_mtx, std::defer_lock},
                 {write_mtx, std::defer_lock}};
    if (read and write) {
      std::lock(rwg.read, rwg.write);
    } else if (read) {
      rwg.read.lock();
    } else if (write) {
      rwg.write.lock();
    }
    return rwg;
  }
};
#endif
}  // namespace pose_graph
}  // namespace vtr
