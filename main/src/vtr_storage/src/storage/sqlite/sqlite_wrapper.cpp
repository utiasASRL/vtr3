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
 * \file sqlite_wrapper.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_storage/storage/sqlite/sqlite_wrapper.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rcutils/types.h"
#include "vtr_storage/serialized_bag_message.hpp"
#include "vtr_storage/storage/sqlite/sqlite_exception.hpp"

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

SqliteWrapper::SqliteWrapper(
  const std::string & uri, storage_interfaces::IOFlag io_flag)
: db_ptr(nullptr)
{
  if (io_flag == storage_interfaces::IOFlag::READ_ONLY) {
    int rc = sqlite3_open_v2(
      uri.c_str(), &db_ptr,
      SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) {
      std::stringstream errmsg;
      errmsg << "Could not read-only open database. SQLite error (" <<
        rc << "): " << sqlite3_errstr(rc) << ". Extended error code: " <<
        sqlite3_extended_errcode(db_ptr);
      throw SqliteException{errmsg.str()};
    }
    // throws an exception if the database is not valid.
    prepare_statement("PRAGMA schema_version;")->execute_and_reset();
  } else {
    int rc = sqlite3_open_v2(
      uri.c_str(), &db_ptr,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) {
      std::stringstream errmsg;
      errmsg << "Could not read-write open database. SQLite error (" <<
        rc << "): " << sqlite3_errstr(rc) << ". Extended error code: " <<
        sqlite3_extended_errcode(db_ptr);
      throw SqliteException{errmsg.str()};
    }
    prepare_statement("PRAGMA journal_mode = WAL;")->execute_and_reset();
    prepare_statement("PRAGMA synchronous = NORMAL;")->execute_and_reset();
  }

  sqlite3_extended_result_codes(db_ptr, 1);
}

SqliteWrapper::SqliteWrapper()
: db_ptr(nullptr) {}

SqliteWrapper::~SqliteWrapper()
{
  const int rc = sqlite3_close(db_ptr);
  if (rc != SQLITE_OK) {
    std::stringstream err;
    err << "ERROR [storage] Could not close open database. Error code: "
        << rc << " Error message: " << sqlite3_errstr(rc);
  }
}

SqliteStatement SqliteWrapper::prepare_statement(const std::string & query)
{
  return std::make_shared<SqliteStatementWrapper>(db_ptr, query);
}

size_t SqliteWrapper::get_last_insert_id()
{
  return sqlite3_last_insert_rowid(db_ptr);
}

SqliteWrapper::operator bool()
{
  return db_ptr != nullptr;
}

// clang-format on

}  // namespace sqlite_storage
}  // namespace storage
}  // namespace vtr
