#pragma once

#include <ostream>
#include <type_traits>

namespace vtr {
namespace common {

// test for the insertion operator between two classes
template <typename S, typename T>
struct has_insertion {
  typedef std::true_type true_t;
  typedef std::false_type false_t;
  template <typename T2>
  static auto test(const T2& t2)
      -> decltype(std::declval<S&&>() << t2, true_t());
  static false_t test(...);  // fallback if S&& << t2 isn't well formed
  typedef decltype(test(std::declval<T&&>())) return_t;
  static const bool value = std::is_same<return_t, std::true_type>::value;
};

template <class Func, class... Args>
struct returns_void {
  typedef std::true_type true_t;
  typedef std::false_type false_t;
  static const bool value =
      std::is_same<void, typename std::result_of<Func(Args...)>::type>::value;
};

namespace safe_stream {

template <typename T>
struct has_os_insertion : has_insertion<std::ostream&, T> {};

template <typename T>
typename std::enable_if<has_os_insertion<T>::value, std::ostream&>::type
insertion(std::ostream& os, const T& t) {
  return os << t;
}
template <typename T>
typename std::enable_if<!has_os_insertion<T>::value, std::ostream&>::type
insertion(std::ostream& os, const T& t) {
  (void)t;
  return os << "(no stream insert)";
}

namespace op {
template <typename T>
typename std::enable_if<!has_os_insertion<T>::value, std::ostream&>::type
operator<<(std::ostream& os, const T& t) {
  return insertion(os, t);
}
}  // namespace op

}  // namespace safe_stream
}  // namespace common
}  // namespace vtr
