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
 * \file sequential_writer.hpp
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

#include "vtr_storage/accessor/base_writer_interface.hpp"
#include "vtr_storage/converter/converter.hpp"
#include "vtr_storage/metadata_io.hpp"
#include "vtr_storage/storage/sqlite/sqlite_storage.hpp"

namespace vtr {
namespace storage {
namespace accessor {

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
/**
 * The Writer allows writing messages to a new bag. For every topic, information about its type
 * needs to be added before writing the first message.
 */
class SequentialWriter
  : public BaseWriterInterface
{
public:
  SequentialWriter();

  ~SequentialWriter() override;

  void open(const std::string & uri) override;

  void reset() override;

  /**
   * Create a new topic in the underlying storage. Needs to be called for every topic used within
   * a message which is passed to write(...).
   *
   * \param topic_with_type name and type identifier of topic to be created
   * \throws runtime_error if the Writer is not open.
   */
  void create_topic(const TopicMetadata & topic_with_type) override;

  /**
   * Remove a new topic in the underlying storage.
   * If creation of subscription fails remove the topic
   * from the db (more of cleanup)
   *
   * \param topic_with_type name and type identifier of topic to be created
   * \throws runtime_error if the Writer is not open.
   */
  void remove_topic(const TopicMetadata & topic_with_type) override;

  /**
   * Write a message to a bagfile. The topic needs to have been created before writing is possible.
   *
   * \param message to be written to the bagfile
   * \throws runtime_error if the Writer is not open.
   */
  void write(std::shared_ptr<SerializedBagMessage> message) override;

  int32_t get_last_inserted_id();

protected:
  std::string base_folder_;
  std::shared_ptr<storage_interfaces::ReadWriteInterface> storage_;
  std::unique_ptr<Converter> converter_;
  std::unique_ptr<MetadataIo> metadata_io_;

  // Used in bagfile splitting; specifies the best-effort maximum sub-section of a bagfile in bytes.
  uint64_t max_bagfile_size_;

  // Intermediate cache to write multiple messages into the storage.
  // `max_cache_size` is the amount of messages to hold in storage before writing to disk.
  uint64_t max_cache_size_;
  std::vector<std::shared_ptr<const SerializedBagMessage>> cache_;

  // Used to track topic -> message count
  std::unordered_map<std::string, TopicInformation> topics_names_to_info_;

  BagMetadata metadata_;

  // Closes the current backed storage and opens the next bagfile.
  void split_bagfile();

  // Checks if the current recording bagfile needs to be split and rolled over to a new file.
  bool should_split_bagfile() const;

  // Prepares the metadata by setting initial values.
  void init_metadata();

  // Record TopicInformation into metadata
  void finalize_metadata();
};
// clang-format on

}  // namespace accessor
}  // namespace storage
}  // namespace vtr