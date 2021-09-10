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
 * \file converter.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_storage/converter/converter.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "vtr_storage/metadata_io.hpp"
#include "vtr_storage/ros_helper.hpp"
#include "vtr_storage/typesupport_helpers.hpp"

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

Converter::Converter()
: input_converter_(std::make_unique<cdr_converter::CdrConverter>()),
  output_converter_(std::make_unique<cdr_converter::CdrConverter>())
{}

Converter::~Converter()
{
  input_converter_.reset();
  output_converter_.reset();
}

std::shared_ptr<SerializedBagMessage> Converter::convert(
  std::shared_ptr<const SerializedBagMessage> message)
{
  auto ts = topics_and_types_.at(message->topic_name).rmw_type_support;
  auto introspection_ts = topics_and_types_.at(message->topic_name).introspection_type_support;
  auto allocator = rcutils_get_default_allocator();
  std::shared_ptr<rosbag2_introspection_message_t> allocated_ros_message =
    allocate_introspection_message(introspection_ts, &allocator);

  input_converter_->deserialize(message, ts, allocated_ros_message);
  auto output_message = std::make_shared<SerializedBagMessage>();
  output_message->serialized_data = make_empty_serialized_message(0);
  output_converter_->serialize(allocated_ros_message, ts, output_message);
  return output_message;
}

void Converter::add_topic(const std::string & topic, const std::string & type)
{
  ConverterTypeSupport type_support;

  type_support.type_support_library = get_typesupport_library(
    type, "rosidl_typesupport_cpp");
  type_support.rmw_type_support = get_typesupport_handle(
    type, "rosidl_typesupport_cpp",
    type_support.type_support_library);

  type_support.introspection_type_support_library = get_typesupport_library(
    type, "rosidl_typesupport_introspection_cpp");
  type_support.introspection_type_support = get_typesupport_handle(
    type, "rosidl_typesupport_introspection_cpp",
    type_support.introspection_type_support_library);

  topics_and_types_.insert({topic, type_support});
}
// clang-format on

}  // namespace storage
}  // namespace vtr