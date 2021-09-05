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
 * \file bag_metadata.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "vtr_storage/storage/topic_metadata.hpp"

namespace vtr {
namespace storage {

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
struct TopicInformation
{
  TopicMetadata topic_metadata;
  size_t message_count;
};

struct BagMetadata
{
  int version = 4;  // upgrade this number when changing the content of the struct
  uint64_t bag_size = 0;  // Will not be serialized
  std::string storage_identifier;
  std::vector<std::string> relative_file_paths;
  std::chrono::nanoseconds duration;
  std::chrono::time_point<std::chrono::high_resolution_clock> starting_time;
  uint64_t message_count;
  std::vector<TopicInformation> topics_with_message_count;
  std::string compression_format;
  std::string compression_mode;
};
// clang-format on

} // namespace storage
} // namespace vtr