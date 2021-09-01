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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rosbag2_cpp/writers/sequential_writer.hpp>

// This is necessary because of using stl types here. It is completely safe,
// because a) the member is not accessible from the outside b) there are no
// inline functions.
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4251)
#endif

namespace vtr {
namespace storage {

class SequentialAppendWriter : public rosbag2_cpp::writers::SequentialWriter {
 public:
  explicit SequentialAppendWriter(
      bool append_mode,
      std::unique_ptr<rosbag2_storage::StorageFactoryInterface>
          storage_factory = std::make_unique<rosbag2_storage::StorageFactory>(),
      std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface>
          converter_factory = std::make_shared<
              rosbag2_cpp::SerializationFormatConverterFactory>(),
      std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
          std::make_unique<rosbag2_storage::MetadataIo>());

  ~SequentialAppendWriter() override;

  /**
   * Opens a new bagfile and prepare it for writing messages if bagfile does not
   *exist. Opens the existing bagfile for appending messages if one exists.
   *
   * \param storage_options Options to configure the storage
   * \param converter_options options to define in which format incoming
   *messages are stored
   **/
  void open(const rosbag2_cpp::StorageOptions& storage_options,
            const rosbag2_cpp::ConverterOptions& converter_options) override;

 protected:
  bool append_mode_ = false;
};
}  // namespace storage
}  // namespace vtr

#ifdef _WIN32
#pragma warning(pop)
#endif
