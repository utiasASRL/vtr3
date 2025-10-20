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
 * \file storage_accessor.hpp
 * \brief StorageAccessor class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <mutex>
#include <unordered_map>

#include "vtr_storage/accessor/base_reader_interface.hpp"
#include "vtr_storage/accessor/base_writer_interface.hpp"
#include "vtr_storage/storage/metadata_io.hpp"
#include "vtr_storage/storage/read_write_interface.hpp"

namespace vtr {
namespace storage {

// clang-format off

/**
 * \brief Storage accessor that handles thread-safe access to the underlying
 * database and metadata writing at destruction.
 * \note Due to the metadata file, having multiple instance of this class is not
 * thread safe.
 */
class StorageAccessor : public BaseReaderInterface, public BaseWriterInterface {
 public:
  ~StorageAccessor() override;

  void open(const std::string &uri, const bool read_only = false) override;
  void close() override;

  /// Reader
  std::shared_ptr<SerializedBagMessage> read_at_timestamp(const Timestamp & timestamp) override;
  std::vector<std::shared_ptr<SerializedBagMessage>> read_at_timestamp_range(const Timestamp & timestamp_begin, const Timestamp & timestamp_end) override;
  std::shared_ptr<SerializedBagMessage> read_at_index(const Index & index) override;
  std::vector<std::shared_ptr<SerializedBagMessage>> read_at_index_range(const Index & index_begin, const Index & index_end) override;

  /// Writer
  void write(const std::shared_ptr<SerializedBagMessage> & message) override;
  void write(const std::vector<std::shared_ptr<SerializedBagMessage>> & messages) override;

  /// Topic filter
  void create_topic(const TopicMetadata &topic_with_type) override;
  void remove_topic(const TopicMetadata &topic_with_type) override;
  void set_filter(const StorageFilter &storage_filter) override;
  void reset_filter() override;

 private:
  std::mutex storage_mutex_; /// protects access to storage_.
  std::string base_folder_;
  std::unique_ptr<ReadWriteInterface> storage_ = nullptr;
  std::unique_ptr<MetadataIo> metadata_io_ = std::make_unique<MetadataIo>();
  std::unordered_map<std::string, TopicInformation> topics_names_to_info_;
};
// clang-format on

}  // namespace storage
}  // namespace vtr