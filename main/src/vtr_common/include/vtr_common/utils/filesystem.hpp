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
 * \file filesystem.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace vtr {
namespace common {
namespace utils {

inline std::string expand_env(std::string path) {
  if (path.find("${") == std::string::npos) return path;

  std::string pre = path.substr(0, path.find("${"));
  std::string post = path.substr(path.find("${") + 2);

  if (post.find('}') == std::string::npos) return path;

  std::string variable = post.substr(0, post.find('}'));
  std::string value = "";

  post = post.substr(post.find('}') + 1);

  const char* v = getenv(variable.c_str());
  if (v != NULL) value = std::string(v);

  return expand_env(pre + value + post);
}

inline std::string expand_user(std::string path) {
  if (path.empty() || path[0] != '~') return path;

  if (path.size() != 1 && path[1] != '/')
    throw std::invalid_argument("Invalid syntax.");

  auto home = getenv("HOME");
  if (home || ((home = getenv("USERPROFILE")))) {
    path.replace(0, 1, home);
  } else {
    auto hdrive = getenv("HOMEDRIVE");
    auto hpath = getenv("HOMEPATH");
    if (!hdrive || !hpath) throw std::invalid_argument("Invalid syntax.");
    path.replace(0, 1, std::string(hdrive) + hpath);
  }
  return path;
}

}  // namespace utils
}  // namespace common
}  // namespace vtr