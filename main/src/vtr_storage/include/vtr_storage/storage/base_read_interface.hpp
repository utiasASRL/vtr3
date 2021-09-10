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
 * \file base_read_interface.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vtr_storage/serialized_bag_message.hpp"
#include "vtr_storage/storage/topic_metadata.hpp"

namespace vtr {
namespace storage {
namespace storage_interfaces {

/// \note the following code is adapted from rosbag2 foxy

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
class BaseReadInterface
{
public:
  virtual ~BaseReadInterface() = default;

  virtual bool has_next() = 0;

  virtual std::shared_ptr<SerializedBagMessage> read_next() = 0;

  virtual std::shared_ptr<SerializedBagMessage>
  read_at_timestamp(rcutils_time_point_value_t timestamp) = 0;

  virtual std::shared_ptr<SerializedBagMessage>
  read_at_index(int32_t index) = 0;

  virtual std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
  read_at_timestamp_range(
    rcutils_time_point_value_t timestamp_begin,
    rcutils_time_point_value_t timestamp_end) = 0;

  virtual std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
  read_at_index_range(int32_t index_begin, int32_t index_end) = 0;

  virtual bool seek_by_index(int32_t index) = 0;

  virtual bool seek_by_timestamp(rcutils_time_point_value_t timestamp) = 0;

  virtual std::shared_ptr<SerializedBagMessage>
  modified_read_next() = 0;

  virtual std::vector<TopicMetadata> get_all_topics_and_types() = 0;

};
// clang-format on

}  // namespace storage_interfaces
}  // namespace storage
}  // namespace vtr