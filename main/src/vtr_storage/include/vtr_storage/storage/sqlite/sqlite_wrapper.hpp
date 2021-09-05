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
 * \file sqlite_wrapper.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <sqlite3.h>

#include <memory>
#include <string>
#include <vector>

#include "rcutils/types.h"
#include "vtr_storage/serialized_bag_message.hpp"
#include "vtr_storage/storage/base_io_interface.hpp"
#include "vtr_storage/storage/sqlite/sqlite_statement_wrapper.hpp"

namespace vtr {
namespace storage {
namespace sqlite_storage {

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

using DBPtr = sqlite3 *;

class SqliteWrapper
{
public:
  SqliteWrapper(const std::string & uri, storage_interfaces::IOFlag io_flag);
  SqliteWrapper();
  ~SqliteWrapper();

  SqliteStatement prepare_statement(const std::string & query);

  size_t get_last_insert_id();

  operator bool();

private:
  DBPtr db_ptr;
};

// clang-format on

}  // namespace sqlite_storage
}  // namespace storage
}  // namespace vtr
