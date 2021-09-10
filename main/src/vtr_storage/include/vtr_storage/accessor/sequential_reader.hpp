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
 * \file sequential_reader.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vtr_storage/accessor/base_reader_interface.hpp"
#include "vtr_storage/converter/converter.hpp"
#include "vtr_storage/metadata_io.hpp"
#include "vtr_storage/storage/sqlite/sqlite_storage.hpp"
#include "vtr_storage/storage/storage_filter.hpp"

namespace vtr {
namespace storage {
namespace accessor {

namespace details {
std::vector<std::string> resolve_relative_paths(
    const std::string& base_folder, std::vector<std::string> relative_files,
    const int version = 4);
}

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
class SequentialReader
  : public BaseReaderInterface
{
public:
  SequentialReader();

  virtual ~SequentialReader();

  void open(const std::string & uri) override;

  void reset() override;

  bool has_next() override;

  std::shared_ptr<SerializedBagMessage> read_next() override;

  const BagMetadata & get_metadata() const override;

  std::vector<TopicMetadata> get_all_topics_and_types() const override;

  void set_filter(const StorageFilter & storage_filter) override;

  void reset_filter() override;

  /**
   * Ask whether there is another database file to read from the list of relative
   * file paths.
   *
   * \return true if there are still files to read in the list
   */
  virtual bool has_next_file() const;

  /**
  * Return the relative file path pointed to by the current file iterator.
  */
  virtual std::string get_current_file() const;

  /**
  * Return the URI of the current file (i.e. no extensions).
  */
  virtual std::string get_current_uri() const;

protected:
  /**
  * Increment the current file iterator to point to the next file in the list of relative file
  * paths.
  *
  * Expected usage:
  * if (has_next_file()) load_next_file();
  */
  virtual void load_next_file();

  /**
    * Fill topics_metadata_ cache vector with information from metadata_
    */
  virtual void fill_topics_metadata();

  std::shared_ptr<storage_interfaces::ReadOnlyInterface> storage_{};
  std::unique_ptr<Converter> converter_{};
  std::unique_ptr<MetadataIo> metadata_io_{};
  BagMetadata metadata_{};
  StorageFilter topics_filter_{};
  std::vector<TopicMetadata> topics_metadata_{};
  std::vector<std::string> file_paths_{};  // List of database files.
  std::vector<std::string>::iterator current_file_iterator_{};  // Index of file to read from
};
// clang-format on

}  // namespace accessor
}  // namespace storage
}  // namespace vtr