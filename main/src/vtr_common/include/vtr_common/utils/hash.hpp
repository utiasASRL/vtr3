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
 * \file hash.hpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <functional>
#include <set>
#include <utility>

namespace vtr {
namespace common {

inline void hash_combine(std::size_t& seed) {}
template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest) {
  seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  hash_combine(seed, rest...);
}

}  // namespace common
}  // namespace vtr

namespace std {
// hash for std::pair
template <class T1, class T2>
struct hash<pair<T1, T2>> {
  size_t operator()(const pair<T1, T2>& p) const {
    size_t seed = 0;
    vtr::common::hash_combine(seed, p.first, p.second);
    return seed;
  }
};

// hash for std::set
template <class T>
struct hash<set<T>> {
  size_t operator()(const set<T>& p) const {
    size_t seed = 0;
    for (const auto& i : p) vtr::common::hash_combine(seed, i);
    return seed;
  }
};

}  // namespace std
