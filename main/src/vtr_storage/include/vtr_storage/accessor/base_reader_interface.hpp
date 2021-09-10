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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <vector>

#include "vtr_storage/serialized_bag_message.hpp"
#include "vtr_storage/storage/bag_metadata.hpp"
#include "vtr_storage/storage/storage_filter.hpp"
#include "vtr_storage/storage/topic_metadata.hpp"

namespace vtr {
namespace storage {
namespace accessor {

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
class BaseReaderInterface {
public:
  virtual ~BaseReaderInterface() {}

  virtual void open(const std::string & uri) = 0;

  virtual void reset() = 0;

  virtual bool has_next() = 0;

  virtual std::shared_ptr<SerializedBagMessage> read_next() = 0;

  virtual const BagMetadata & get_metadata() const = 0;

  virtual std::vector<TopicMetadata> get_all_topics_and_types() const = 0;

  virtual void set_filter(const StorageFilter & storage_filter) = 0;

  virtual void reset_filter() = 0;
};
// clang-format on

}  // namespace accessor
}  // namespace storage
}  // namespace vtr