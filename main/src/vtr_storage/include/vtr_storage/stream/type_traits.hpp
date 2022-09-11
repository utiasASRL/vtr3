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
 * \file type_traits.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <type_traits>

namespace vtr {
namespace storage {

template <typename T, typename Enabled = void>
struct has_to_storable {
  static constexpr bool value = false;
  // if it is not a storable object, then it has to be a ros2 message
  typedef T type;
};

template <typename T>
struct has_to_storable<T, std::enable_if_t<std::is_member_function_pointer_v<
                              decltype(&T::toStorable)>>> {
  static constexpr bool value =
      std::is_member_function_pointer_v<decltype(&T::toStorable)>;
  typedef typename std::result_of<decltype (&T::toStorable)(T)>::type type;
};

template <typename T, typename Enabled = void>
struct has_from_storable {
  static constexpr bool value = false;
};

template <typename T>
struct has_from_storable<
    T, std::enable_if_t<std::is_function_v<decltype(T::fromStorable)>>> {
  static constexpr bool value = std::is_function_v<decltype(T::fromStorable)>;
};

template <typename T>
struct is_storable {
  static constexpr bool value =
      has_from_storable<T>::value && has_to_storable<T>::value;
};

}  // namespace storage
}  // namespace vtr