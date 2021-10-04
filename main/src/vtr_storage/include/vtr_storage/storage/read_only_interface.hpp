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
 * \file read_only_interface.hpp
 * \brief ReadOnlyInterface class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <string>

#include "vtr_storage/storage/base_info_interface.hpp"
#include "vtr_storage/storage/base_io_interface.hpp"
#include "vtr_storage/storage/base_read_interface.hpp"
#include "vtr_storage/storage/storage_filter.hpp"

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

class ReadOnlyInterface
  : public BaseInfoInterface, public BaseIOInterface, public BaseReadInterface
{
public:
  virtual ~ReadOnlyInterface() = default;

  void open(const std::string & uri, IOFlag io_flag = IOFlag::READ_ONLY) override = 0;

  uint64_t get_bagfile_size() const override = 0;

  std::string get_storage_identifier() const override = 0;

  /**
  Sets filters on messages. This occurs in place, meaning that messages satisfying
  the filter that were already read before applying the filter will not be re-read
  by read_next() unless seek(t) is also called to an earlier timestamp t.
  */
  virtual void set_filter(const StorageFilter & storage_filter) = 0;

  /**
  Removes any previously set storage filter. This occurs in place, meaning that
  after a reset, read_next() will not return either previously read or unread
  messages that occur before the timestamp of the last-read message.
  */
  virtual void reset_filter() = 0;

  /**
  Seeks to a given timestamp. Running read_next() after seek(t)
  will return a message that is equal to or after time t. Running read_next()
  repeatedly until the end of the storage should return all messages equal to
  or after time t.

  If a filter has been previously set, it will persist, meaning
  that the next message returned will need to both satisfy the filter,
  and satisfy the seek time requirement.

  seek(t) can jump forward or backward in time. If t is earlier than the
  first message message timestamp that satisfies the storage filter, then read_next()
  will return that first message. If t is later than the last message timestamp, then
  read_next() will behave as if the end of the file has been reached, and has_next()
  will return false.
  */
  virtual void seek(const rcutils_time_point_value_t & timestamp) = 0;
};
// clang-format on

}  // namespace storage
}  // namespace vtr
