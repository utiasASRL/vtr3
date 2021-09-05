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
 * \file serialization_format_serializer.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "vtr_storage/converter/introspection_message.hpp"

#include "vtr_storage/serialized_bag_message.hpp"

namespace vtr {
namespace storage {
namespace converter_interfaces {

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
class SerializationFormatSerializer
{
public:
  virtual ~SerializationFormatSerializer() = default;

  virtual void serialize(
    std::shared_ptr<const rosbag2_introspection_message_t> ros_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<SerializedBagMessage> serialized_message) = 0;
};
// clang-format on

} // namespace converter_interfaces
} // namespace storage
} // namespace vtr
