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
 * \file sequential_append_writer.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"

#include <vtr_storage/accessor/sequential_append_writer.hpp>

namespace vtr {
namespace storage {
namespace accessor {

std::string format_storage_uri(const std::string& base_folder,
                               uint64_t storage_count) {
  // Right now `base_folder_` is always just the folder name for where to
  // install the bagfile. The name of the folder needs to be queried in case
  // SequentialWriter is opened with a relative path.
  std::stringstream storage_file_name;
  storage_file_name << rcpputils::fs::path(base_folder).filename().string()
                    << "_" << storage_count;

  return (rcpputils::fs::path(base_folder) / storage_file_name.str()).string();
}

SequentialAppendWriter::SequentialAppendWriter(bool append_mode)
    : append_mode_(append_mode) {}

void SequentialAppendWriter::open(const std::string& uri) {
  base_folder_ = uri;
  max_bagfile_size_ = 0;  // max_bagfile_size = 0 means never split
  max_cache_size_ = 0;    // max_cache_size = 0 means no caching

  cache_.reserve(max_cache_size_);

  rcpputils::fs::path db_path(base_folder_);
  // will fail if file already exists, i.e. in append mode
  rcpputils::fs::create_directories(db_path);

  const auto storage_uri = format_storage_uri(base_folder_, 0);
  if (!append_mode_) {
    rcpputils::fs::remove(rcpputils::fs::path(storage_uri + ".db3"));
    rcpputils::fs::remove(rcpputils::fs::path(storage_uri + ".db3-shm"));
    rcpputils::fs::remove(rcpputils::fs::path(storage_uri + ".db3-wal"));
    rcpputils::fs::remove(db_path / "metadata.yaml");
  }

  storage_ = std::make_shared<sqlite_storage::SqliteStorage>();
  storage_->open(storage_uri, storage_interfaces::IOFlag::READ_WRITE);

  if (max_bagfile_size_ != 0 &&
      max_bagfile_size_ < storage_->get_minimum_split_file_size()) {
    std::stringstream error;
    error << "Invalid bag splitting size given. Please provide a value greater "
             "than "
          << storage_->get_minimum_split_file_size() << ". Specified value of "
          << max_bagfile_size_;
    throw std::runtime_error{error.str()};
  }

  if (append_mode_) {
    // get information on existing message topics, and add to the writer's
    // metadata and converter metadata_ = storage_->get_metadata();
    metadata_ = metadata_io_->read_metadata(uri);
    std::cout << "DEBUG [storage] Message count before appending: "
              << metadata_.message_count << std::endl;
    storage_->get_all_topics_and_types();
    for (const auto& topic_information : metadata_.topics_with_message_count) {
      const auto insert_res = topics_names_to_info_.insert(std::make_pair(
          topic_information.topic_metadata.name, topic_information));

      if (!insert_res.second) {
        std::stringstream errmsg;
        errmsg << "Failed to insert topic \""
               << topic_information.topic_metadata.name << "\"!";

        throw std::runtime_error(errmsg.str());
      }
      if (converter_) {
        converter_->add_topic(topic_information.topic_metadata.name,
                              topic_information.topic_metadata.type);
      }
    }
    std::cout << "DEBUG [storage] Number of topics before appending: "
              << topics_names_to_info_.size() << std::endl;
  } else {
    init_metadata();
  }
}

}  // namespace accessor
}  // namespace storage
}  // namespace vtr