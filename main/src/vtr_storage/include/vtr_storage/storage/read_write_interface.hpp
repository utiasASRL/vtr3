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
 * \file read_write_interface.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <string>

#include "vtr_storage/storage/base_write_interface.hpp"
#include "vtr_storage/storage/read_only_interface.hpp"
#include "vtr_storage/storage/storage_filter.hpp"

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
class ReadWriteInterface
  : public ReadOnlyInterface, public BaseWriteInterface
{
public:
  ~ReadWriteInterface() override = default;

  void open(const std::string & uri, IOFlag io_flag = IOFlag::READ_WRITE) override = 0;

  uint64_t get_bagfile_size() const override = 0;

  std::string get_storage_identifier() const override = 0;

  virtual uint64_t get_minimum_split_file_size() const = 0;

  void set_filter(const StorageFilter & storage_filter) override = 0;

  void reset_filter() override = 0;
};
// clang-format on

}  // namespace storage_interfaces
}  // namespace storage
}  // namespace vtr