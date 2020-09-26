#pragma once

#include <memory>
#include <mutex>

#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

namespace vtr {
namespace pose_graph {

struct RosBagIO {
  using DataStreamReaderBasePtr =
      std::shared_ptr<storage::DataStreamReaderBase>;
  using DataStreamWriterBasePtr =
      std::shared_ptr<storage::DataStreamWriterBase>;

  DataStreamReaderBasePtr first;
  DataStreamWriterBasePtr second;
  std::recursive_mutex read_mtx;
  std::recursive_mutex write_mtx;

  using Guard = std::unique_lock<std::recursive_mutex>;
  struct RWGuard {
    Guard read, write;
  };
  RWGuard lock(bool read = true, bool write = true) {
    RWGuard rwg{{read_mtx, std::defer_lock}, {write_mtx, std::defer_lock}};
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
}  // namespace pose_graph
}  // namespace vtr
