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
 * \file base_reader_interface.hpp
 * \brief BaseReaderInterface class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <vector>

#include "vtr_storage/storage/bag_metadata.hpp"
#include "vtr_storage/storage/serialized_bag_message.hpp"
#include "vtr_storage/storage/storage_filter.hpp"
#include "vtr_storage/storage/topic_metadata.hpp"

namespace vtr {
namespace storage {

/// \note the following code is adapted from rosbag2 galactic

// Copyright 2018, Bosch Software Innovations GmbH.
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

// clang-format off
class BaseReaderInterface {
public:
  using Index = int;
  using Timestamp = rcutils_time_point_value_t;

  virtual ~BaseReaderInterface() {}

  virtual void open(const std::string & uri) = 0;
  virtual void close() = 0;

  virtual std::shared_ptr<SerializedBagMessage> read_at_timestamp(const Timestamp & timestamp) = 0;
  virtual std::vector<std::shared_ptr<SerializedBagMessage>> read_at_timestamp_range(const Timestamp & timestamp_begin, const Timestamp & timestamp_end) = 0;
  virtual std::shared_ptr<SerializedBagMessage> read_at_index(const Index & index) = 0;
  virtual std::vector<std::shared_ptr<SerializedBagMessage>> read_at_index_range(const Index & index_begin, const Index & index_end) = 0;

  virtual void set_filter(const StorageFilter & storage_filter) = 0;
  virtual void reset_filter() = 0;
};
// clang-format on

}  // namespace storage
}  // namespace vtr