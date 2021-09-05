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
 * \file converter.hpp
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

#include "rcpputils/shared_library.hpp"

#include "vtr_storage/converter/cdr/cdr_converter.hpp"
#include "vtr_storage/converter/serialization_format_converter.hpp"
#include "vtr_storage/serialized_bag_message.hpp"

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

// Convenience struct to keep both type supports (rmw and introspection) together.
// Only used internally.
struct ConverterTypeSupport
{
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library;
  const rosidl_message_type_support_t * rmw_type_support;

  std::shared_ptr<rcpputils::SharedLibrary> introspection_type_support_library;
  const rosidl_message_type_support_t * introspection_type_support;
};

class Converter
{
public:
  Converter();
  ~Converter();

  /**
   * Converts the given SerializedBagMessage into the output format of the converter. The
   * serialization format of the input message must be identical to the input format of the
   * converter.
   *
   * \param message Message to convert
   * \returns Converted message
   */
  std::shared_ptr<SerializedBagMessage>
  convert(std::shared_ptr<const SerializedBagMessage> message);

  void add_topic(const std::string & topic, const std::string & type);

private:
  std::unique_ptr<converter_interfaces::SerializationFormatDeserializer> input_converter_;
  std::unique_ptr<converter_interfaces::SerializationFormatSerializer> output_converter_;
  std::unordered_map<std::string, ConverterTypeSupport> topics_and_types_;
};
// clang-format on

}  // namespace storage
}  // namespace vtr