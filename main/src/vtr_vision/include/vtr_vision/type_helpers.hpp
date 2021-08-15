/**
 * \file type_helpers.hpp
 * \brief Convenience header for outlier rejection (RANSAC)
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <ostream>

#include <vtr_messages/msg/matches.hpp>
#include <vtr_vision/types.hpp>

inline std::ostream& operator<<(std::ostream& os,
                                const vtr::vision::SimpleMatch& m) {
  return os << "(" << m.first << "," << m.second << ")";
}

inline std::ostream& operator<<(std::ostream& os,
                                const vtr::vision::SimpleMatches& ms) {
  for (auto& m : ms) os << m << " ";
  return os;
}

inline bool operator==(const vtr::vision::SimpleMatch& a,
                       const vtr::vision::SimpleMatch& b) {
  return a.first == b.first && a.second == b.second;
}

inline std::ostream& operator<<(std::ostream& os,
                                const vtr_messages::msg::FeatureId& id) {
  return os << "(" << id.idx << "," << id.channel << "," << id.camera << ","
            << id.rig << "," << id.persistent.robot << id.persistent.stamp
            << ")";
}

inline std::ostream& operator<<(std::ostream& os,
                                const vtr_messages::msg::Match& match) {
  return os << match.from_id << "-> ()";
  for (const auto& idx : match.to_id) {
    os << idx << ",";
  }
  os << ")";
}
/*
inline std::ostream& operator<<(std::ostream& os,
                                const asrl::vision_msgs::Matches & ms) {
  for (const auto & m : ms.matches()) os << m << " ";
  return os;
} */

namespace vtr {
namespace vision {

/*inline std::ostream& operator<<(std::ostream& os,
                                const vtr_vision::vision::Match & p) {
  os << "(" << p.from << ",(";
  for (auto & t : p.to) os << t << ",";
  return os << "\b)";
}*/

/*
inline bool operator==(const vtr_vision::vision::Match & a,
                       const vtr_vision::vision::Match & b) {
  return (a.from == b.from) && (a.to == b.to);
}*/

}  // namespace vision
}  // namespace vtr
