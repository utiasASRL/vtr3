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
 * \file sequential_append_writer.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_storage/accessor/sequential_writer.hpp>

namespace vtr {
namespace storage {

namespace accessor {

class SequentialAppendWriter : public SequentialWriter {
 public:
  explicit SequentialAppendWriter(bool append_mode);

  virtual ~SequentialAppendWriter() {}

  /**
   * \brief Opens a new bagfile and prepare it for writing messages if bagfile
   * does not exist. Opens the existing bagfile for appending messages if one
   * exists.
   */
  void open(const std::string& uri) override;

 protected:
  bool append_mode_ = false;
};

}  // namespace accessor
}  // namespace storage
}  // namespace vtr