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
 * \file cache.hpp
 * \brief QueryCache class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include <lgmath.hpp>
#include <steam.hpp>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

template <class DataType>
class Cache {
 public:
  using CacheType = Cache<DataType>;
  using DataPtr = std::shared_ptr<DataType>;

  Cache() = default;
  Cache(const Cache&) = delete;
  Cache(Cache&&) = delete;
  Cache& operator=(const Cache&) = delete;
  Cache& operator=(Cache&& other) = delete;

  bool valid() const { return datum_ != nullptr; };
  explicit operator bool() const { return valid(); }

  CacheType& clear() { datum_.reset(); }

  template <typename... Args>
  CacheType& emplace(Args&&... args) {
    datum_ = std::make_shared<DataType>(std::forward<Args>(args)...);
    return *this;
  }

  template <class T = DataType>
  typename std::enable_if<std::is_copy_assignable<T>::value, CacheType&>::type
  operator=(const DataType& datum) {
    if (valid())
      *datum_ = datum;
    else
      datum_ = std::make_shared<DataType>(datum);
    return *this;
  }

  template <class T = DataType>
  typename std::enable_if<std::is_move_assignable<T>::value, CacheType&>::type
  operator=(DataType&& datum) {
    if (valid())
      *datum_ = std::move(datum);
    else
      datum_ = std::make_shared<DataType>(std::move(datum));
    return *this;
  }

  CacheType& operator=(const DataPtr& datum) {
    datum_ = datum;
    return *this;
  }

  CacheType& operator=(DataPtr&& datum) {
    datum_ = std::move(datum);
    return *this;
  }

  const DataType& operator*() const {
    if (valid())
      return *datum_;
    else
      throw std::runtime_error("cache datum is unset on reference request.");
  }
  DataType& operator*() {
    return const_cast<DataType&>(*static_cast<const CacheType&>(*this));
  }
  const DataType* operator->() const { return &(this->operator*()); }
  DataType* operator->() {
    return const_cast<DataType*>(
        static_cast<const CacheType&>(*this).operator->());
  }

  const DataPtr& ptr() const { return datum_; }
  DataPtr& ptr() { return datum_; }

 private:
  DataPtr datum_;
};

template <class DataType>
using LockableCache = common::SharedLockable<Cache<DataType>>;

struct QueryCache {
  using Ptr = std::shared_ptr<QueryCache>;

  virtual ~QueryCache() = default;

  Cache<rclcpp::Node> node;
  Cache<rclcpp::Time> rcl_stamp;
  Cache<storage::Timestamp> stamp;
  Cache<PipelineMode> pipeline_mode;
  Cache<bool> first_frame;
  Cache<VertexId> live_id;
  Cache<EdgeTransform> T_r_m_odo;
  Cache<steam::se3::SteamTrajInterface> trajectory;
  Cache<KeyframeTestResult> keyframe_test_result;
  Cache<bool> odo_success;
  Cache<VertexId> map_id;
  Cache<LocalizationChain> loc_chain;
  Cache<EdgeTransform> T_r_m_loc;
  Cache<bool> loc_success;
  Cache<std::string> robot_frame;
};

}  // namespace tactic
}  // namespace vtr