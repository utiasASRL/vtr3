#pragma once

#include "rclcpp/rclcpp.hpp"

#include <lgmath.hpp>
#include <steam.hpp>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_tactic/utils/cache_container.hpp>

#include <vtr_messages/msg/time_stamp.hpp>

namespace vtr {
namespace tactic {

/// \todo Should replace this class with an unordered_map? The overhead for data
/// accessing should be negligible.
struct QueryCache : public common::CacheContainer {
  using Ptr = std::shared_ptr<QueryCache>;

  QueryCache()
      : node("node", janitor_.get()),
        rcl_stamp("rcl_stamp", janitor_.get()),
        stamp("stamp", janitor_.get()),
        first_frame("first_frame", janitor_.get()),
        live_id("live_id", janitor_.get()),
        map_id("map_id", janitor_.get()),
        T_r_m_odo("T_r_m_odo", janitor_.get()),
        T_r_m_loc("T_r_m_loc", janitor_.get()),
        keyframe_test_result("keyframe_test_result", janitor_.get()),
        odo_success("odo_success", janitor_.get()),
        loc_success("loc_success", janitor_.get()),
        trajectory("trajectory", janitor_.get()),
        robot_frame("robot_frame", janitor_.get()) {}

  virtual ~QueryCache() = default;

  common::cache_ptr<rclcpp::Node> node;
  common::cache_ptr<rclcpp::Time> rcl_stamp;
  common::cache_ptr<vtr_messages::msg::TimeStamp> stamp;
  common::cache_ptr<bool> first_frame;
  common::cache_ptr<VertexId> live_id;
  common::cache_ptr<VertexId> map_id;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_odo;  //
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_loc;  //
  common::cache_ptr<KeyframeTestResult> keyframe_test_result;
  common::cache_ptr<bool, true> odo_success;
  common::cache_ptr<bool, true> loc_success;
  common::cache_ptr<steam::se3::SteamTrajInterface> trajectory;
  common::cache_ptr<std::string> robot_frame;
};
}  // namespace tactic
}  // namespace vtr