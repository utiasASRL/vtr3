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
 * \file container_tools.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <deque>

namespace vtr {
namespace common {
namespace utils {

template <class C>
bool contains(const C &container, const typename C::key_type &item) {
  auto result = container.find(item);
  return (result != container.end());
}

template <class C>
std::deque<std::reference_wrapper<typename C::value_type>> getRefs(
    C &container) {
  std::deque<std::reference_wrapper<typename C::value_type>> d;
  for (auto &&it : container) {
    d.emplace_back(it);
  }
  return d;
}

template <class C>
std::deque<std::reference_wrapper<const typename C::value_type>> getRefs(
    const C &container) {
  std::deque<std::reference_wrapper<const typename C::value_type>> d;
  for (auto &&it : container) {
    d.emplace_back(it);
  }
  return d;
}

}  // namespace utils
}  // namespace common
}  // namespace vtr
