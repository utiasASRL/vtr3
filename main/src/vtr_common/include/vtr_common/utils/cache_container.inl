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
 * \file cache_container.inl
 * \brief Implements the reference-needed template functions.
 * \details They are hidden in this file to allow the cache classes to be
 * declared with incomplete types.
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/utils/cache_container.hpp>

namespace vtr {
namespace common {

///// @brief the ostream operator overload, calls the virtual member
std::ostream& operator<<(std::ostream& os, const cache_base& me) {
  return me << os;  // yes, it looks weird, it's calling the overloaded member
}

template <typename Type>
std::ostream& cache_accessor<Type>::operator<<(std::ostream& os) const {
  // return os << *this;
  name_.empty() ? os << "(noname)" : os << name_;
  os << ": ";
  using namespace safe_stream::op;
  is_valid() ? os << operator*()
             : os << "(empty!)";  // safe_stream << in case T doesn't have <<
  return os;
}

template <typename Type, bool Guaranteed>
auto cache_ptr<Type, Guaranteed>::operator=(Type&& datum) -> my_t& {
  return assign(std::move(datum));
}
template <typename Type, bool Guaranteed>
auto cache_ptr<Type, Guaranteed>::operator=(const Type& datum) -> my_t& {
  return assign(datum);
}

template <typename Type, bool Guaranteed>
auto cache_ptr<Type, Guaranteed>::fallback(const Type& datum) -> my_t& {
  return fallback<const Type&>(datum);
}
template <typename Type, bool Guaranteed>
auto cache_ptr<Type, Guaranteed>::fallback(Type&& datum) -> my_t& {
  return fallback<Type&&>(std::move(datum));
}

}  // namespace common
}  // namespace vtr
