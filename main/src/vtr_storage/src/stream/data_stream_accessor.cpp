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
 * \file data_stream_accessor.cpp
 * \brief DataStreamAccessor(Base) class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_storage/stream/data_stream_accessor.hpp>

namespace vtr {
namespace storage {

DataStreamAccessorBase::DataStreamAccessorBase(
    const std::string &base_directory, const std::string &stream_name,
    const std::string &stream_type)
    : base_directory_(std::filesystem::path(base_directory)),
      data_directory_(std::filesystem::path(base_directory) / stream_name) {
  storage_accessor_->open(data_directory_.string());

  tm_.name = stream_name;
  tm_.type = stream_type;
  tm_.serialization_format = "cdr";
  storage_accessor_->create_topic(tm_);
}

}  // namespace storage
}  // namespace vtr