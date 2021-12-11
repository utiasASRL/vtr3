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
 * \file rc_graph.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/graph.hpp"
#include "vtr_pose_graph/serializable/rc_graph_base.hpp"

#include "vtr_pose_graph_msgs/msg/graph.hpp"
#include "vtr_pose_graph_msgs/msg/map_info.hpp"
#include "vtr_pose_graph_msgs/msg/timestamp.hpp"

namespace vtr {
namespace pose_graph {

// Virtual inheritance is necessary to ensure that relevant methods in GraphBase
// stay overridden
class RCGraph : public RCGraphBase, public Graph<RCVertex, RCEdge, RCRun> {
 public:
  PTR_TYPEDEFS(RCGraph);

  using GraphType = Graph<RCVertex, RCEdge, RCRun>;
  using Base = RCGraphBase;
  using RType = RCGraphBase;

  using VertexType = typename Base::VertexType;
  using VertexPtr = typename Base::VertexPtr;
  using VertexIdType = typename Base::VertexIdType;
  using SimpleVertexId = typename Base::SimpleVertexId;

  using EdgeType = typename Base::EdgeType;
  using EdgePtr = typename Base::EdgePtr;
  using EdgeIdType = typename Base::EdgeIdType;
  using EdgeEnumType = typename Base::EdgeEnumType;
  using SimpleEdgeId = typename Base::SimpleEdgeId;
  using TransformType = typename Base::TransformType;

  using RunType = typename Base::RunType;
  using RunPtr = typename Base::RunPtr;
  using RunIdType = typename Base::RunIdType;

  using Base::edges_;
  using Base::graph_;
  using Base::runs_;
  using Base::vertices_;
  using GraphType::mtx_;

  using RunFilter = std::unordered_set<RunIdType>;

  using GraphMsg = vtr_pose_graph_msgs::msg::Graph;
  using MapInfoMsg = vtr_pose_graph_msgs::msg::MapInfo;

  using GraphMsgAccessor = storage::DataStreamAccessor<GraphMsg>;
  using RunMsgAccessor = storage::DataStreamAccessor<RCRun::RunMsg>;

  /** \brief Pseudo constructor for making shared pointers */
  static Ptr MakeShared(
      const std::string& file_path, const bool load = true,
      const CallbackPtr& callback = std::make_shared<Callback>()) {
    return std::make_shared<RCGraph>(file_path, load, callback);
  }

  /** \brief Construct an empty graph with an id and save location */
  RCGraph(const std::string& file_path, const bool load = true,
          const CallbackPtr& callback = std::make_shared<Callback>());
  RCGraph(const RCGraph&) = delete;
  RCGraph(RCGraph&& other) = delete;
  RCGraph& operator=(const RCGraph&) = delete;
  RCGraph& operator=(RCGraph&& other) = delete;

  virtual ~RCGraph() { save(); }

  void save();

  /** \brief Add a new run an increment the run id */
  RunIdType addRun();

  /** \brief Return a blank vertex with the next available Id */
  VertexPtr addVertex(const Timestamp& time, const RunIdType& run_id);

  /** \brief Return a blank vertex(current run) with the next available Id */
  VertexPtr addVertex(const Timestamp& time) {
    LockGuard lck(mtx_);
    return addVertex(time, current_run_->id());
  }

  /** \brief Get the map display calibration */
  MapInfoMsg getMapInfo() const {
    LockGuard lck(mtx_);
    return map_info_;
  }

  /** \brief Set the map display calibration */
  void setMapInfo(const MapInfoMsg& map_info) {
    LockGuard lck(mtx_);
    map_info_ = map_info;
  }

  /** \brief Get the file path of the graph index */
  std::string filePath() const { return file_path_; }

 private:
  /** \brief Load the top-level index from file */
  void loadGraphIndex();
  /** \brief Deep load runs and their vertex/edge data */
  void loadRuns(const RunFilter& r = RunFilter());
  /** \brief Save the top-level index to file */
  void saveGraphIndex();
  /** \brief Save all modified runs to file */
  void saveRuns();
  /**
   * \brief Builds the simple graph using loaded vertices and edges, and add
   * neighbor information to vertices_.
   */
  void buildSimpleGraphAndAddVertexNeighbors();
  /** \brief Build map from persistent ids to existing vertex ids */
  void buildPersistentMap();

  const std::string file_path_;

  /** \brief Ros message containing necessary information for a list of runs. */
  storage::LockableMessage<GraphMsg>::Ptr msg_ = nullptr;

  MapInfoMsg map_info_ = MapInfoMsg();
};

}  // namespace pose_graph
}  // namespace vtr