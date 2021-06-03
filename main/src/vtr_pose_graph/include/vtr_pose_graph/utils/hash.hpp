#pragma once

#include <vtr_common/utils/hash.hpp>

namespace std {
// Create a hash for std::pair so that it can be used as a key for unordered
// containers (e.g. std::unordered_map). If this hurts performance so much,
// consider changing it back to boost::unordered_map.
template <class T1, class T2>
struct hash<pair<T1, T2>> {
  size_t operator()(const pair<T1, T2>& p) const {
    size_t seed = 0;
    vtr::common::hash_combine(seed, p.first, p.second);
    return seed;
  }
};
}  // namespace std