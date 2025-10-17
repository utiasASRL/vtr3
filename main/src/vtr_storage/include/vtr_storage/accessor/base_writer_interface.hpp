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
 * \file base_writer_interface.hpp
 * \brief BaseWriterInterface class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include "vtr_storage/storage/serialized_bag_message.hpp"
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
class BaseWriterInterface
{
public:
  virtual ~BaseWriterInterface() {}

  virtual void open(const std::string & uri, const bool read_only) = 0;
  virtual void close() = 0;

  virtual void write(const std::shared_ptr<SerializedBagMessage> & message) = 0;
  virtual void write(const std::vector<std::shared_ptr<SerializedBagMessage>> & messages) = 0;

  virtual void create_topic(const TopicMetadata & topic_with_type) = 0;
  virtual void remove_topic(const TopicMetadata & topic_with_type) = 0;
};
// clang-format on

}  // namespace storage
}  // namespace vtr