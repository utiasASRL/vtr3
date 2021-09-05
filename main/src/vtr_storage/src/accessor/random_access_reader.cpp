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
 * \file random_access_reader.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/asserts.hpp"
#include "rcpputils/filesystem_helper.hpp"

#include <vtr_storage/accessor/random_access_reader.hpp>

namespace vtr {
namespace storage {
namespace accessor {
RandomAccessReader::RandomAccessReader(const std::string &stream_name)
    : stream_name_(stream_name) {}

std::shared_ptr<SerializedBagMessage> RandomAccessReader::read_at_timestamp(
    rcutils_time_point_value_t timestamp) {
  if (storage_) {
    auto message = storage_->read_at_timestamp(timestamp);
    return (converter_ && message) ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

void RandomAccessReader::open(const std::string &uri) {
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
    /// \todo this is an ugly hack that handles case where a stream write has
    /// not been closed so there is no metadata saved.
    file_paths_ = details::resolve_relative_paths(
        uri, {uri + ("/" + stream_name_ + "_0.db3")}, 4);
    current_file_iterator_ = file_paths_.begin();

    storage_ = std::make_shared<sqlite_storage::SqliteStorage>();
    storage_->open(get_current_file(), storage_interfaces::IOFlag::READ_ONLY);

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

std::shared_ptr<SerializedBagMessage> RandomAccessReader::read_at_index(
    int32_t index) {
  if (storage_) {
    auto message = storage_->read_at_index(index);
    return (converter_ && message) ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
RandomAccessReader::read_at_timestamp_range(
    rcutils_time_point_value_t timestamp_begin,
    rcutils_time_point_value_t timestamp_end) {
  if (storage_) {
    auto message_vector =
        storage_->read_at_timestamp_range(timestamp_begin, timestamp_end);
    if (converter_) {
      for (auto &message : *message_vector) {
        if (message) {
          message = converter_->convert(message);
        }
      }
    }
    return message_vector;
  } else {
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }
}
std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
RandomAccessReader::read_at_index_range(int32_t index_begin,
                                        int32_t index_end) {
  if (storage_) {
    auto message_vector = storage_->read_at_index_range(index_begin, index_end);
    if (converter_) {
      for (auto &message : *message_vector) {
        if (message) {
          message = converter_->convert(message);
        }
      }
    }
    return message_vector;
  } else {
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }
}

bool RandomAccessReader::seek_by_index(int32_t index) {
  return storage_ && storage_->seek_by_index(index);
}

bool RandomAccessReader::seek_by_timestamp(
    rcutils_time_point_value_t timestamp) {
  return storage_ && storage_->seek_by_timestamp(timestamp);
}

std::shared_ptr<SerializedBagMessage>
RandomAccessReader::read_next_from_seek() {
  if (storage_) {
    auto message = storage_->modified_read_next();
    return (converter_ && message) ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

}  // namespace accessor
}  // namespace storage
}  // namespace vtr