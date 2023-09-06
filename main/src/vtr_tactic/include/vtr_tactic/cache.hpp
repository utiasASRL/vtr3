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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "steam.hpp"

#include "vtr_tactic/types.hpp"

#include "vtr_common/utils/hash.hpp" // For pair hash function in obstacle map

namespace vtr {
namespace tactic {

template <class DataType>
class Cache {
 public:
  using CacheType = Cache<DataType>;
  using DataPtr = std::shared_ptr<DataType>;

  bool valid() const { return datum_ != nullptr; };
  explicit operator bool() const { return valid(); }

  CacheType& clear() {
    datum_.reset();
    return *this;
  }

  template <typename... Args>
  CacheType& emplace(Args&&... args) {
    if (datum_ != nullptr) throw std::runtime_error("Cache already has data");
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
    else {
      throw std::runtime_error("cache datum is unset on reference request. Cache type is: " + type_name(datum_));
    }
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
  DataPtr datum_ = nullptr;
};

template <class DataType>
using LockableCache = common::SharedLockable<Cache<DataType>>;

struct QueryCache : std::enable_shared_from_this<QueryCache> {
  using Ptr = std::shared_ptr<QueryCache>;

  virtual ~QueryCache() = default;

  // input
  Cache<rclcpp::Node> node;
  Cache<Timestamp> stamp;
  Cache<EnvInfo> env_info;

  // preprocessing
  Cache<const PipelineMode> pipeline_mode;
  Cache<const bool> first_frame;

  // odometry and mapping - initialized in tactic, modified by pipeline
  Cache<VertexId> vid_odo;
  Cache<VertexTestResult> vertex_test_result;
  Cache<bool> odo_success;
  Cache<EdgeTransform> T_r_v_odo;
  Cache<Eigen::Matrix<double, 6, 1>> w_v_r_in_r_odo;  // body centric velocity

  // localization
  Cache<const VertexId> vid_loc;
  Cache<const unsigned> sid_loc;
  Cache<EdgeTransform> T_r_v_loc;
  Cache<bool> loc_success;

  // graph memory management cache args
  Cache<const VertexId> live_mem_async;
  Cache<const std::pair<VertexId, VertexId>> graph_mem_async;

};

/** \brief Shared memory to the path tracker. */
struct OutputCache : std::enable_shared_from_this<OutputCache> {
  using Ptr = std::shared_ptr<OutputCache>;

  virtual ~OutputCache() = default;

  Cache<rclcpp::Node> node;
  Cache<LocalizationChain> chain;

  // Obstacle Costmap:
  using XY2ValueMap = std::unordered_map<std::pair<float, float>, float>;
  XY2ValueMap obs_map;
  unsigned costmap_sid;
  float grid_resolution = 0.25;
};

}  // namespace tactic
}  // namespace vtr