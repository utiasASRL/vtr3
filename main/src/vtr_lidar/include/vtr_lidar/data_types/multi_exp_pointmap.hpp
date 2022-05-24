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
 * \file multi_exp_pointmap.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_types/pointmap.hpp"

#include "vtr_lidar_msgs/msg/multi_exp_point_map.hpp"

namespace vtr {
namespace lidar {

template <class PointT>
class MultiExpPointMap : public PointMap<PointT> {
 public:
  using typename PointScan<PointT>::PointCloudType;
  PTR_TYPEDEFS(MultiExpPointMap<PointT>);

  using MultiExpPointMapMsg = vtr_lidar_msgs::msg::MultiExpPointMap;
  /** \brief Static function that constructs this class from ROS2 message */
  static Ptr fromStorable(const MultiExpPointMapMsg& storable);
  /** \brief Returns the ROS2 message to be stored */
  MultiExpPointMapMsg toStorable() const;

  MultiExpPointMap(const float& dl, const size_t& max_num_exps);

  size_t max_num_exps() const { return max_num_exps_; }

  std::deque<uint32_t>& exps() { return exps_; }
  const std::deque<uint32_t>& exps() const { return exps_; }

 private:
  /** \brief Maximum number of experiences */
  size_t max_num_exps_;
  /** \brief Experience Id vector */
  std::deque<uint32_t> exps_;
};

}  // namespace lidar
}  // namespace vtr

#include "vtr_lidar/data_types/multi_exp_pointmap.inl"