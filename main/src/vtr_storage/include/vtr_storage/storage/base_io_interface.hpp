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
 * \file base_io_interface.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <string>

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
enum class IOFlag : uint8_t
{
  READ_ONLY = 0,
  READ_WRITE = 1,
  APPEND = 2
};

// When bagfile splitting feature is not enabled or applicable,
// use 0 as the default maximum bagfile size value.
extern const uint64_t MAX_BAGFILE_SIZE_NO_SPLIT;

class BaseIOInterface
{
public:
  virtual ~BaseIOInterface() = default;

  /**
   * Opens the storage plugin.
   * \param uri is the path to the bagfile. Exact behavior depends on the io_flag passed.
   * \param io_flag is a hint for the type of storage plugin to open depending on the io operations requested.
   * If IOFlag::READ_ONLY is passed, then only read operations are guaranteed.
   * The uri passed should be the exact relative path to the bagfile.
   * If IOFlag::READ_WRITE is passed, then a new bagfile is created with guaranteed read and write operations.
   * The storage plugin will append the uri in the case of creating a new bagfile backing.
   */
  virtual void open(const std::string & uri, IOFlag io_flag) = 0;
};
// clang-format on

}  // namespace storage_interfaces
}  // namespace storage
}  // namespace vtr