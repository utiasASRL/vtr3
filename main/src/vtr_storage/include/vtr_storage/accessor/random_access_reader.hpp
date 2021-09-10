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
 * \file random_access_reader.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_storage/accessor/sequential_reader.hpp>

namespace vtr {
namespace storage {
namespace accessor {

class RandomAccessReader : public SequentialReader {
 public:
  RandomAccessReader(const std::string &stream_name);

  virtual ~RandomAccessReader() {}

  void open(const std::string &uri) override;

  std::shared_ptr<SerializedBagMessage> read_at_timestamp(
      rcutils_time_point_value_t timestamp);

  std::shared_ptr<SerializedBagMessage> read_at_index(int32_t index);

  std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
  read_at_timestamp_range(rcutils_time_point_value_t timestamp_begin,
                          rcutils_time_point_value_t timestamp_end);

  std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
  read_at_index_range(int32_t index_begin, int32_t index_end);

  bool seek_by_index(int32_t index);

  bool seek_by_timestamp(rcutils_time_point_value_t timestamp);

  std::shared_ptr<SerializedBagMessage> read_next_from_seek();

 protected:
  std::string stream_name_;
};

}  // namespace accessor
}  // namespace storage
}  // namespace vtr
