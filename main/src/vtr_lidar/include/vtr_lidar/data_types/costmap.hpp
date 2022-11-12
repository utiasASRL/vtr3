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
 * \file costmap.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <algorithm>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_common/conversions/ros_lgmath.hpp"
#include "vtr_common/utils/hash.hpp"  // for std::pair hash
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace lidar {
namespace costmap {

struct PixKey {
  PixKey(int x0 = 0, int y0 = 0) : x(x0), y(y0) {}
  // clang-format off
  bool operator==(const PixKey& other) const { return (x == other.x && y == other.y); }
  friend PixKey operator+(const PixKey A, const PixKey B) { return PixKey(A.x + B.x, A.y + B.y); }
  friend PixKey operator-(const PixKey A, const PixKey B) { return PixKey(A.x - B.x, A.y - B.y); }
  // clang-format on
  int x, y;
};

}  // namespace costmap
}  // namespace lidar
}  // namespace vtr

// Specialization of std:hash function
namespace std {

template <>
struct hash<vtr::lidar::costmap::PixKey> {
  std::size_t operator()(const vtr::lidar::costmap::PixKey& k) const {
    std::size_t ret = 0;
    vtr::common::hash_combine(ret, k.x, k.y);
    return ret;
  }
};

}  // namespace std

namespace vtr {
namespace lidar {

class BaseCostMap {
 public:
  BaseCostMap(const float& dl, const float& size_x, const float& size_y,
              const float& default_value = 0);
  virtual ~BaseCostMap() = default;

 public:
  using XY2ValueMap = std::unordered_map<std::pair<float, float>, float>;
  virtual XY2ValueMap filter(const float& threshold) const = 0;

 public:
  float dl() const { return dl_; }
  virtual tactic::EdgeTransform& T_vertex_this() { return T_vertex_this_; }
  const tactic::EdgeTransform& T_vertex_this() const { return T_vertex_this_; }
  virtual tactic::VertexId& vertex_id() { return vertex_id_; }
  const tactic::VertexId& vertex_id() const { return vertex_id_; }
  virtual unsigned& vertex_sid() { return vertex_sid_; }
  const unsigned& vertex_sid() const { return vertex_sid_; }

  virtual tactic::EdgeTransform& T_this_plan() { return T_this_plan_; }
  const tactic::EdgeTransform& T_this_plan() const { return T_this_plan_; }

 protected:
  template <typename PointT>
  costmap::PixKey getKey(const PointT& p) const;
  template <typename PointT>
  bool contains(const PointT& k) const;
  bool contains(const costmap::PixKey& k) const;
  virtual float at(const costmap::PixKey& k) const = 0;

 protected:
  /** \brief cost map resolution */
  const float dl_;
  /** \brief size of the cost map in x and y axis */
  const float size_x_, size_y_;
  /** \brief default value of the cost map */
  const float default_value_;
  /** \brief number of cells in x (width) and y (height) direction */
  const int width_, height_;
  /** \brief pixel key corresponds to the lower left corner (x: right, y: up) */
  const costmap::PixKey origin_;

  /// Pipeline related
  /** \brief the associated vertex sequence id */
  unsigned vertex_sid_ = -1;
  /** \brief the associated vertex id */
  tactic::VertexId vertex_id_ = tactic::VertexId::Invalid();
  /** \brief the transform from this scan/map to its associated vertex */
  tactic::EdgeTransform T_vertex_this_ = tactic::EdgeTransform(true);

  /// Planning related
  /** \brief the transform from the planning frame to the this costmap */
  tactic::EdgeTransform T_this_plan_ = tactic::EdgeTransform(true);
};

class DenseCostMap : public BaseCostMap {
 public:
  DenseCostMap(const float& dl, const float& size_x, const float& size_y,
               const float& default_value = 0);

  /**
   * \brief Iterates over all cells in the cost map, calls ComputeValueOp to get
   * a cost.
   */
  template <typename ComputeValueOp>
  void update(const ComputeValueOp& op);

  /** \brief update from a sparse cost map */
  void update(const std::unordered_map<costmap::PixKey, float>& values);

  XY2ValueMap filter(const float& threshold) const override;

  /** \brief Returns content of this class as a OccupancyGrid message. */
  using CostMapMsg = nav_msgs::msg::OccupancyGrid;
  CostMapMsg toCostMapMsg() const;

  /** \brief Returns content of this class as a PointCloud2 message. */
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  PointCloudMsg toPointCloudMsg() const;

 protected:
  float at(const costmap::PixKey& k) const override;

 private:
  Eigen::MatrixXf values_;
};

class SparseCostMap : public BaseCostMap {
 public:
  /**
   * \param[in] dl resolution of the grid
   * \param[in] size_x total size of the grid in x direction [meter]
   * \param[in] size_y total size of the grid in y direction [meter]
   */
  SparseCostMap(const float& dl, const float& size_x, const float& size_y,
                const float& default_value = 0);

  struct AvgOp {
    using InputIt = std::vector<float>::const_iterator;
    float operator()(InputIt first, InputIt last) const {
      float count = 0.;
      float sum = 0.;
      for (; first != last; ++first) {
        sum += *first;
        ++count;
      }
      return count == 0 ? 0.0 : sum / count;
    }
  };
  template <typename PointCloud, typename ReductionOp = AvgOp>
  void update(const PointCloud& points, const std::vector<float>& values,
              const ReductionOp& op = ReductionOp());

  DenseCostMap toDense() const;

  XY2ValueMap filter(const float& threshold) const override;

  /** \brief Returns content of this class as a OccupancyGrid message. */
  using CostMapMsg = nav_msgs::msg::OccupancyGrid;
  CostMapMsg toCostMapMsg() const;

  /** \brief Returns content of this class as a PointCloud2 message. */
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  PointCloudMsg toPointCloudMsg() const;

 protected:
  float at(const costmap::PixKey& k) const override;

 private:
  std::unordered_map<costmap::PixKey, std::vector<float>> raw_values_;
  std::unordered_map<costmap::PixKey, float> values_;
};

}  // namespace lidar
}  // namespace vtr

#include "vtr_lidar/data_types/costmap.inl"