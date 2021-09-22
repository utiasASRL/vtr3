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
 * \file metadata_io.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_storage/metadata_io.hpp"

#include <fstream>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/filesystem.h"

#include "vtr_storage/storage/topic_metadata.hpp"

#ifdef _WIN32
// This is necessary because of a bug in yaml-cpp's cmake
#define YAML_CPP_DLL
// This is necessary because yaml-cpp does not always use dllimport/dllexport
// consistently
#pragma warning(push)
#pragma warning(disable : 4251)
#pragma warning(disable : 4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
#pragma warning(pop)
#endif

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
namespace YAML
{

using namespace vtr::storage;

/// Pass metadata version to the sub-structs of BagMetadata for deserializing.
/**
  * Encoding should always use the current metadata version, so it does not need this value.
  * We cannot extend the YAML::Node class to include this, so we must call it
  * as a function with the node as an argument.
  */
template<typename T>
T decode_for_version(const Node & node, int version)
{
  static_assert(
    std::is_default_constructible<T>::value,
    "Type passed to decode_for_version that has is not default constructible.");
  if (!node.IsDefined()) {
    throw TypedBadConversion<T>(node.Mark());
  }
  T value{};
  if (convert<T>::decode(node, value, version)) {
    return value;
  }
  throw TypedBadConversion<T>(node.Mark());
}

template<>
struct convert<TopicMetadata>
{
  static Node encode(const TopicMetadata & topic)
  {
    Node node;
    node["name"] = topic.name;
    node["type"] = topic.type;
    node["serialization_format"] = topic.serialization_format;
    node["offered_qos_profiles"] = topic.offered_qos_profiles;
    return node;
  }

  static bool decode(const Node & node, TopicMetadata & topic, int version)
  {
    topic.name = node["name"].as<std::string>();
    topic.type = node["type"].as<std::string>();
    topic.serialization_format = node["serialization_format"].as<std::string>();
    if (version >= 4) {
      topic.offered_qos_profiles = node["offered_qos_profiles"].as<std::string>();
    } else {
      topic.offered_qos_profiles = "";
    }
    return true;
  }
};

template<>
struct convert<TopicInformation>
{
  static Node encode(const TopicInformation & metadata)
  {
    Node node;
    node["topic_metadata"] = metadata.topic_metadata;
    node["message_count"] = metadata.message_count;
    return node;
  }

  static bool decode(const Node & node, TopicInformation & metadata, int version)
  {
    metadata.topic_metadata = decode_for_version<TopicMetadata>(
      node["topic_metadata"], version);
    metadata.message_count = node["message_count"].as<uint64_t>();
    return true;
  }
};

template<>
struct convert<std::vector<TopicInformation>>
{
  static Node encode(const std::vector<TopicInformation> & rhs)
  {
    Node node{NodeType::Sequence};
    for (const auto & value : rhs) {
      node.push_back(value);
    }
    return node;
  }

  static bool decode(
    const Node & node, std::vector<TopicInformation> & rhs, int version)
  {
    if (!node.IsSequence()) {
      return false;
    }

    rhs.clear();
    for (const auto & value : node) {
      rhs.push_back(decode_for_version<TopicInformation>(value, version));
    }
    return true;
  }
};

template<>
struct convert<std::chrono::nanoseconds>
{
  static Node encode(const std::chrono::nanoseconds & time_in_ns)
  {
    Node node;
    node["nanoseconds"] = time_in_ns.count();
    return node;
  }

  static bool decode(const Node & node, std::chrono::nanoseconds & time_in_ns)
  {
    time_in_ns = std::chrono::nanoseconds(node["nanoseconds"].as<uint64_t>());
    return true;
  }
};

template<>
struct convert<std::chrono::time_point<std::chrono::high_resolution_clock>>
{
  static Node encode(const std::chrono::time_point<std::chrono::high_resolution_clock> & start_time)
  {
    Node node;
    node["nanoseconds_since_epoch"] = start_time.time_since_epoch().count();
    return node;
  }

  static bool decode(
    const Node & node, std::chrono::time_point<std::chrono::high_resolution_clock> & start_time)
  {
    start_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds(node["nanoseconds_since_epoch"].as<uint64_t>()));
    return true;
  }
};

template<>
struct convert<BagMetadata>
{
  static Node encode(const BagMetadata & metadata)
  {
    Node node;
    node["version"] = metadata.version;
    node["storage_identifier"] = metadata.storage_identifier;
    node["relative_file_paths"] = metadata.relative_file_paths;
    node["duration"] = metadata.duration;
    node["starting_time"] = metadata.starting_time;
    node["message_count"] = metadata.message_count;
    node["topics_with_message_count"] = metadata.topics_with_message_count;

    if (metadata.version >= 3) {  // fields introduced by rosbag2_compression
      node["compression_format"] = metadata.compression_format;
      node["compression_mode"] = metadata.compression_mode;
    }
    return node;
  }

  static bool decode(const Node & node, BagMetadata & metadata)
  {
    metadata.version = node["version"].as<int>();
    metadata.storage_identifier = node["storage_identifier"].as<std::string>();
    metadata.relative_file_paths = node["relative_file_paths"].as<std::vector<std::string>>();
    metadata.duration = node["duration"].as<std::chrono::nanoseconds>();
    metadata.starting_time = node["starting_time"]
      .as<std::chrono::time_point<std::chrono::high_resolution_clock>>();
    metadata.message_count = node["message_count"].as<uint64_t>();
    metadata.topics_with_message_count =
      decode_for_version<std::vector<TopicInformation>>(
      node["topics_with_message_count"], metadata.version);

    if (metadata.version >= 3) {  // fields introduced by rosbag2_compression
      metadata.compression_format = node["compression_format"].as<std::string>();
      metadata.compression_mode = node["compression_mode"].as<std::string>();
    }
    return true;
  }
};

}  // namespace YAML
// clang-format on

namespace vtr {
namespace storage {

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
void MetadataIo::write_metadata(const std::string & uri, const BagMetadata & metadata)
{
  YAML::Node metadata_node;
  metadata_node["rosbag2_bagfile_information"] = metadata;
  std::ofstream fout(get_metadata_file_name(uri));
  fout << metadata_node;
}

BagMetadata MetadataIo::read_metadata(const std::string & uri)
{
  try {
    YAML::Node yaml_file = YAML::LoadFile(get_metadata_file_name(uri));
    auto metadata = yaml_file["rosbag2_bagfile_information"].as<BagMetadata>();
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
#ifdef VTR_ROS_FOXY
    // ROS2 foxy api
    metadata.bag_size = rcutils_calculate_directory_size(uri.c_str(), allocator);
#else
    // ROS2 >=galactic api
    if (RCUTILS_RET_OK !=
      rcutils_calculate_directory_size(uri.c_str(), &metadata.bag_size, allocator))
    {
      throw std::runtime_error(
        std::string("Exception on calculating the size of directory :") + uri);
    }
#endif
    return metadata;
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error(std::string("Exception on parsing info file: ") + ex.what());
  }
}

std::string MetadataIo::get_metadata_file_name(const std::string & uri)
{
  std::string metadata_file = (rcpputils::fs::path(uri) / metadata_filename).string();

  return metadata_file;
}

bool MetadataIo::metadata_file_exists(const std::string & uri)
{
  return rcpputils::fs::path(get_metadata_file_name(uri)).exists();
}
// clang-format on

}  // namespace storage
}  // namespace vtr
