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
 * \file sequential_reader.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_storage/accessor/sequential_reader.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/asserts.hpp"
#include "rcpputils/filesystem_helper.hpp"

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
namespace details
{
std::vector<std::string> resolve_relative_paths(
  const std::string & base_folder, std::vector<std::string> relative_files, const int version)
{
  auto base_path = rcpputils::fs::path(base_folder);
  if (version < 4) {
    // In older rosbags (version <=3) relative files are prefixed with the rosbag folder name
    base_path = rcpputils::fs::path(base_folder).parent_path();
  }

  rcpputils::require_true(
    base_path.exists(), "base folder does not exist: " + base_folder);
  rcpputils::require_true(
    base_path.is_directory(), "base folder has to be a directory: " + base_folder);

  for (auto & file : relative_files) {
    auto path = rcpputils::fs::path(file);
    if (path.is_absolute()) {
      continue;
    }
    file = (base_path / path).string();
  }

  return relative_files;
}
}  // namespace details

SequentialReader::SequentialReader()
: converter_(nullptr),
  metadata_io_(std::make_unique<MetadataIo>())
{}

SequentialReader::~SequentialReader()
{
  reset();
}

void SequentialReader::reset()
{
  if (storage_) {
    storage_.reset();
  }
}

void SequentialReader::open(const std::string & uri)
{
  // If there is a metadata.yaml file present, load it.
  // If not, let's ask the storage with the given URI for its metadata.
  // This is necessary for non ROS2 bags (aka ROS1 legacy bags).
  if (metadata_io_->metadata_file_exists(uri)) {
    metadata_ = metadata_io_->read_metadata(uri);
    if (metadata_.relative_file_paths.empty()) {
      std::cout << "WARNING [storage] No file paths were found in metadata.";
      return;
    }

    file_paths_ = details::resolve_relative_paths(
      uri, metadata_.relative_file_paths, metadata_.version);
    current_file_iterator_ = file_paths_.begin();

    storage_ = std::make_shared<sqlite_storage::SqliteStorage>();
    storage_->open(get_current_file(), storage_interfaces::IOFlag::READ_ONLY);
  } else {
    /// \todo this block is for rosbag1 and is no longer valid. In ros2, the uri
    /// in `storage_->open(uri, flag)` must always get from metadata file
    /// because it has to be appended by one of the relative_file_paths in
    /// metadata. Remove this block in later clean up.
    storage_ = std::make_shared<sqlite_storage::SqliteStorage>();
    storage_->open(uri, storage_interfaces::IOFlag::READ_ONLY);

    metadata_ = storage_->get_metadata();
    if (metadata_.relative_file_paths.empty()) {
      std::cout << "WARNING [storage] No file paths were found in metadata.";
      return;
    }
    file_paths_ = metadata_.relative_file_paths;
    current_file_iterator_ = file_paths_.begin();
  }
  auto topics = metadata_.topics_with_message_count;
  if (topics.empty()) {
    std::cout << "WARNING [storage] No topics were listed in metadata.";
    return;
  }
  fill_topics_metadata();
}

bool SequentialReader::has_next()
{
  if (storage_) {
    // If there's no new message, check if there's at least another file to read and update storage
    // to read from there. Otherwise, check if there's another message.
    if (!storage_->has_next() && has_next_file()) {
      load_next_file();

      storage_ = std::make_shared<sqlite_storage::SqliteStorage>();
      storage_->open(get_current_file(), storage_interfaces::IOFlag::READ_ONLY);
      storage_->set_filter(topics_filter_);
    }

    return storage_->has_next();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<SerializedBagMessage> SequentialReader::read_next()
{
  if (storage_) {
    auto message = storage_->read_next();
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

const BagMetadata & SequentialReader::get_metadata() const
{
  rcpputils::check_true(storage_ != nullptr, "Bag is not open. Call open() before reading.");
  return metadata_;
}

std::vector<TopicMetadata> SequentialReader::get_all_topics_and_types() const
{
  rcpputils::check_true(storage_ != nullptr, "Bag is not open. Call open() before reading.");
  return topics_metadata_;
}

void SequentialReader::set_filter(
  const StorageFilter & storage_filter)
{
  topics_filter_ = storage_filter;
  if (storage_) {
    storage_->set_filter(topics_filter_);
    return;
  }
  throw std::runtime_error(
          "Bag is not open. Call open() before setting filter.");
}

void SequentialReader::reset_filter()
{
  topics_filter_ = StorageFilter();
  if (storage_) {
    storage_->reset_filter();
    return;
  }
  throw std::runtime_error(
          "Bag is not open. Call open() before resetting filter.");
}

bool SequentialReader::has_next_file() const
{
  return current_file_iterator_ + 1 != file_paths_.end();
}

void SequentialReader::load_next_file()
{
  assert(current_file_iterator_ != file_paths_.end());
  current_file_iterator_++;
}

std::string SequentialReader::get_current_file() const
{
  return *current_file_iterator_;
}

std::string SequentialReader::get_current_uri() const
{
  auto current_file = get_current_file();
  auto current_uri = rcpputils::fs::remove_extension(current_file);
  return current_uri.string();
}

void SequentialReader::fill_topics_metadata()
{
  rcpputils::check_true(storage_ != nullptr, "Bag is not open. Call open() before reading.");
  topics_metadata_.clear();
  topics_metadata_.reserve(metadata_.topics_with_message_count.size());
  for (const auto & topic_information : metadata_.topics_with_message_count) {
    topics_metadata_.push_back(topic_information.topic_metadata);
  }
}
// clang-format on
}  // namespace accessor
}  // namespace storage
}  // namespace vtr