/// This file implements the reference-needed template functions.
/// They are hidden in this file to allow the cache classes to be declared with
/// incomplete types.
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
