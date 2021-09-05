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
 * \file introspection_message.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include "rcutils/allocator.h"
#include "rcutils/time.h"

struct rosidl_message_type_support_t;

namespace rosidl_typesupport_introspection_cpp {
struct MessageMembers;
}

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
typedef struct rosbag2_introspection_message_t
{
  void * message;
  char * topic_name;
  rcutils_time_point_value_t time_stamp;
  rcutils_allocator_t allocator;
} rosbag2_introspection_message_t;

std::shared_ptr<rosbag2_introspection_message_t>
allocate_introspection_message(
  const rosidl_message_type_support_t * introspection_ts, const rcutils_allocator_t * allocator);

void introspection_message_set_topic_name(
  rosbag2_introspection_message_t * msg, const char * topic_name);

void allocate_internal_types(
  void * msg, const rosidl_typesupport_introspection_cpp::MessageMembers * members);

void deallocate_introspection_message(
  rosbag2_introspection_message_t * msg,
  const rosidl_message_type_support_t * introspection_ts);
// clang-format on

}  // namespace storage
}  // namespace vtr
