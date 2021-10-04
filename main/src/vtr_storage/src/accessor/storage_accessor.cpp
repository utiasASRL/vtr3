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
 * \file storage_accessor.cpp
 * \brief StorageAccessor class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_storage/accessor/storage_accessor.hpp"

#include <filesystem>

#include "rcpputils/asserts.hpp"
#include "rcpputils/filesystem_helper.hpp"

#include "vtr_storage/storage/sqlite/sqlite_storage.hpp"

namespace fs = std::filesystem;

namespace {

std::string strip_parent_path(const std::string& relative_path) {
  return rcpputils::fs::path(relative_path).filename().string();
}

}  // namespace

namespace vtr {
namespace storage {

StorageAccessor::~StorageAccessor() { close(); }

void StorageAccessor::open(const std::string& uri) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);

  if (storage_ != nullptr && base_folder_ != uri)
    throw std::runtime_error(
        "Storage accessor has been opened with a different directory.");

  rcpputils::fs::path db_path(uri);
  bool dir_created = rcpputils::fs::create_directories(db_path);
  if (!dir_created) {
    std::stringstream error;
    error << "Failed to create database directory (" << uri << ").";
    throw std::runtime_error{error.str()};
  }

  base_folder_ = uri;

  // get database path assuming uri is a directory
  const std::string relative_file_path =
      fs::path(uri + "/").parent_path().filename().string() + "_0.db3";

  storage_ = std::make_unique<sqlite::SqliteStorage>();
  storage_->open(base_folder_ + "/" + relative_file_path, IOFlag::READ_WRITE);

  if (!metadata_io_->metadata_file_exists(uri)) return;

  /// Handle case where metadata exists, meaning there's already a bag
  const auto metadata = metadata_io_->read_metadata(uri);

  // sanity check
  rcpputils::check_true(metadata.relative_file_paths.size() == 1,
                        "VTR storage does not support multiple bag files.");
  rcpputils::check_true(metadata.relative_file_paths[0] == relative_file_path,
                        "Inconsistent database path name detected.");

  for (const auto& topic_info : metadata.topics_with_message_count) {
    const auto insert_res = topics_names_to_info_.insert(
        std::make_pair(topic_info.topic_metadata.name, topic_info));
    if (!insert_res.second) {
      std::stringstream errmsg;
      errmsg << "Failed to insert topic \"" << topic_info.topic_metadata.name
             << "\"!";
      throw std::runtime_error(errmsg.str());
    }
  }
}

void StorageAccessor::close() {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);

  if (!storage_) return;

  if (!base_folder_.empty()) {
    auto metadata = storage_->get_metadata();
    // get_metadata function returns full path to the database file path, but
    // ros2 bag seems to require relative file paths
    std::vector<std::string> relative_file_paths;
    for (const auto& file_path : metadata.relative_file_paths)
      relative_file_paths.push_back(strip_parent_path(file_path));
    metadata.relative_file_paths = relative_file_paths;
    metadata_io_->write_metadata(base_folder_, metadata);
  }
  base_folder_.clear();
  storage_.reset();
  topics_names_to_info_.clear();
}

std::shared_ptr<SerializedBagMessage> StorageAccessor::read_at_timestamp(
    const Timestamp& timestamp) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  return storage_->read_at_timestamp(timestamp);
}

std::vector<std::shared_ptr<SerializedBagMessage>>
StorageAccessor::read_at_timestamp_range(const Timestamp& timestamp_begin,
                                         const Timestamp& timestamp_end) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  return storage_->read_at_timestamp_range(timestamp_begin, timestamp_end);
}

std::shared_ptr<SerializedBagMessage> StorageAccessor::read_at_index(
    const Index& index) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  return storage_->read_at_index(index);
}

std::vector<std::shared_ptr<SerializedBagMessage>>
StorageAccessor::read_at_index_range(const Index& index_begin,
                                     const Index& index_end) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  return storage_->read_at_index_range(index_begin, index_end);
}

void StorageAccessor::write(
    const std::shared_ptr<SerializedBagMessage>& message) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before writing.");

  // Update the message count for the Topic.
  if (message->index == 0)
    ++topics_names_to_info_.at(message->topic_name).message_count;

  storage_->write(message);
}

void StorageAccessor::write(
    const std::vector<std::shared_ptr<SerializedBagMessage>>& messages) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before writing.");

  // Update the message count for the Topic.
  for (const auto& message : messages) {
    if (message->index == 0)
      ++topics_names_to_info_.at(message->topic_name).message_count;
  }

  storage_->write(messages);
}

void StorageAccessor::create_topic(const TopicMetadata& topic_with_type) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before writing.");

  if (topics_names_to_info_.find(topic_with_type.name) !=
      topics_names_to_info_.end())
    return;

  TopicInformation info;
  info.topic_metadata = topic_with_type;
  info.message_count = 0;

  const auto insert_res =
      topics_names_to_info_.insert(std::make_pair(topic_with_type.name, info));

  if (!insert_res.second) {
    std::stringstream errmsg;
    errmsg << "Failed to insert topic \"" << topic_with_type.name << "\"!";
    throw std::runtime_error(errmsg.str());
  }

  storage_->create_topic(topic_with_type);
}

void StorageAccessor::remove_topic(const TopicMetadata& topic_with_type) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error("Bag is not open. Call open() before removing.");

  if (topics_names_to_info_.erase(topic_with_type.name) > 0) {
    storage_->remove_topic(topic_with_type);
  } else {
    std::stringstream errmsg;
    errmsg << "Failed to remove the non-existing topic \""
           << topic_with_type.name << "\"!";
    throw std::runtime_error(errmsg.str());
  }
}

void StorageAccessor::set_filter(const StorageFilter& storage_filter) {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (storage_)
    throw std::runtime_error(
        "Bag is not open. Call open() before setting filter.");
  storage_->set_filter(storage_filter);
}

void StorageAccessor::reset_filter() {
  std::lock_guard<std::mutex> storage_lock(storage_mutex_);
  if (!storage_)
    throw std::runtime_error(
        "Bag is not open. Call open() before resetting filter.");
  storage_->reset_filter();
}

}  // namespace storage
}  // namespace vtr