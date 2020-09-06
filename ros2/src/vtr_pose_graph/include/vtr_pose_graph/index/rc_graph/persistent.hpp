#pragma once

#include <functional>

#include <vtr_common/utils/hash.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_messages/msg/graph_persistent_id.hpp>

namespace vtr {
namespace pose_graph {

/// class PersistentIdHasher {
///  public:
///   std::size_t operator()(const graph_msgs::PersistentId &persistent_id)
///   const {
///     std::size_t seed = 0;
///     common::hash_combine(seed, persistent_id.stamp(),
///     persistent_id.robot()); return seed;
///   }
/// };

struct PersistentIdHasher {
  std::size_t operator()(
      const vtr_messages::msg::GraphPersistentId &persistent_id) const {
    std::size_t seed = 0;
    common::hash_combine(seed, persistent_id.stamp, persistent_id.robot);
    return seed;
  }
};

}  // namespace pose_graph
}  // namespace vtr

namespace std {

/// template <>
/// struct equal_to<asrl::graph_msgs::PersistentId> {
///   bool operator()(const asrl::graph_msgs::PersistentId &a,
///                   const asrl::graph_msgs::PersistentId &b) const {
///     // Check validity
///     if (!(a.has_robot() && a.has_stamp() && b.has_robot() && b.has_stamp()))
///     {
///       LOG(ERROR) << "Invalid persistent id\n"
///                  << a.DebugString() << "\n"
///                  << b.DebugString() << "\n"
///                  << el::base::debug::StackTrace();
///       throw std::invalid_argument("invalid persistent id");
///     }
///     return a.robot() == b.robot() && a.stamp() == b.stamp();
///   }
/// };
using PersistentId = vtr_messages::msg::GraphPersistentId;
template <>
struct equal_to<PersistentId> {
  bool operator()(const PersistentId &a, const PersistentId &b) const {
    return a.robot == b.robot && a.stamp == b.stamp;
  }
};

}  // namespace std
