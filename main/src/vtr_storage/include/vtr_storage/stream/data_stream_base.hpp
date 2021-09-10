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
 * \file data_stream_base.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rcpputils/filesystem_helper.hpp"

namespace vtr {
namespace storage {

class DataStreamBase {
 public:
  DataStreamBase(const std::string &base_directory,
                 const std::string &stream_name = "");
  virtual ~DataStreamBase() {}

 protected:
  rcpputils::fs::path base_directory_;
  rcpputils::fs::path data_directory_;
  std::string stream_name_;
  bool opened_;
};
}  // namespace storage
}  // namespace vtr
