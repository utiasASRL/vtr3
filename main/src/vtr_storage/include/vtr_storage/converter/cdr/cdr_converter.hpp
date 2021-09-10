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
 * \file cdr_converter.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <string>

#include "rmw/types.h"

#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

#include "rcpputils/shared_library.hpp"

#include "vtr_storage/converter/introspection_message.hpp"
#include "vtr_storage/converter/serialization_format_converter.hpp"

namespace vtr {
namespace storage {
namespace cdr_converter {

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
class CdrConverter : public converter_interfaces::SerializationFormatConverter
{
public:
  CdrConverter();

  void deserialize(
    std::shared_ptr<const SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_introspection_message_t> introspection_message) override;

  void serialize(
    std::shared_ptr<const rosbag2_introspection_message_t> introspection_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<SerializedBagMessage> serialized_message) override;

private:
  std::shared_ptr<rcpputils::SharedLibrary> library;

protected:
  rmw_ret_t (* serialize_fcn_)(
    const void *,
    const rosidl_message_type_support_t *,
    rmw_serialized_message_t *) = nullptr;

  rmw_ret_t (* deserialize_fcn_)(
    const rmw_serialized_message_t *,
    const rosidl_message_type_support_t *,
    void *) = nullptr;
};
// clang-format on

} // namespace cdr_converter
} // namespace storage
} // namespace vtr