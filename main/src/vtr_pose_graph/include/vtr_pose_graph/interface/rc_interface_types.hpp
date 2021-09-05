// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file rc_interface_types.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <mutex>

#include <vtr_storage/stream/data_stream_reader.hpp>
#include <vtr_storage/stream/data_stream_writer.hpp>

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
