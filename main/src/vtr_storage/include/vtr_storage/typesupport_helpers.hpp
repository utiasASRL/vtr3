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
 * \file typesupport_helpers.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "rcpputils/shared_library.hpp"

#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

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
std::shared_ptr<rcpputils::SharedLibrary>
get_typesupport_library(const std::string & type, const std::string & typesupport_identifier);

const rosidl_message_type_support_t *
get_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  std::shared_ptr<rcpputils::SharedLibrary> library);

const std::tuple<std::string, std::string, std::string>
extract_type_identifier(const std::string & full_type);
// clang-format on

}  // namespace storage
}  // namespace vtr
