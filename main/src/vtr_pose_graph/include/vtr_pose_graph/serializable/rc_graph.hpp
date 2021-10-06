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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/graph.hpp>
#include <vtr_pose_graph/serializable/rc_graph_base.hpp>

#include <vtr_pose_graph_msgs/msg/graph.hpp>
#include <vtr_pose_graph_msgs/msg/map_info.hpp>
#include <vtr_pose_graph_msgs/msg/timestamp.hpp>

namespace vtr {
namespace pose_graph {

// Virtual inheritance is necessary to ensure that relevant methods in GraphBase
// stay overridden
class RCGraph : public RCGraphBase, public Graph<RCVertex, RCEdge, RCRun> {
 public:
  using GraphType = Graph<RCVertex, RCEdge, RCRun>;
  using RType = RCGraphBase;

  using GraphType::Base;
  using GraphType::mtx_;

  using Base::edges_;
  using Base::graph_;
  using Base::runs_;
  using Base::vertices_;

  using RunFilter = std::unordered_set<RunIdType>;

  using GraphMsg = vtr_pose_graph_msgs::msg::Graph;
  using MapInfoMsg = vtr_pose_graph_msgs::msg::MapInfo;

  using GraphMsgAccessor = storage::DataStreamAccessor<GraphMsg>;
  using RunMsgAccessor = storage::DataStreamAccessor<RCRun::RunMsg>;

  PTR_TYPEDEFS(RCGraph);

  /** \brief Pseudo constructor for making shared pointers */
  static Ptr MakeShared(const std::string& file_path, const bool load = true) {
    return std::make_shared<RCGraph>(file_path, load);
  }

  /** \brief Construct an empty graph with an id and save location */
  RCGraph(const std::string& file_path, const bool load = true);

  /// \note Yuchen: we used to allow copying and moving, but I don't think it is
  /// needed or even safe to do so.
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

  /** \brief Get the file path of the graph index */
  std::string filePath() const { return file_path_; }

  /** \brief registers a stream to a run. */
  template <typename DataType>
  void registerVertexStream(const RunIdType& run_id,
                            const std::string& stream_name);

  /** \brief Get the map display calibration */
  /// \todo yuchen is this safe?
  const MapInfoMsg& mapInfo() const { return map_info_; }

  /** \brief Get the map display calibration */
  /// \todo yuchen is this safe?
  MapInfoMsg& mutableMapInfo() { return map_info_; }

  /** \brief Determine if a display map has been set for this graph */
  bool hasMap() const { return map_info_.set; }

  /** \brief Set the map display calibration */
  void setMapInfo(const MapInfoMsg& map_info) {
    map_info_ = map_info;
    map_info_.set = true;  // manually set to true in case we forget it in argin
  }

  /** \brief Remove map information from a graph (USE CAREFULLY) */
  void clearMap() { map_info_.set = false; }

 private:
#if false
  /**
   * \brief Add a new run an increment the run id
   * \details This function is disabled for RCGraphs....
   */
  RunIdType addRun() {
    std::stringstream ss;
    ss << "addRun(robotId) must be called for RCGraphs\n"
       << el::base::debug::StackTrace();
    throw std::runtime_error(ss.str());
    return RunIdType(-1);
  }

  // Disable this function, since we need to know the timestamp
  VertexPtr addVertex() {
    std::stringstream ss;
    ss << "Must provide timestamps for RCVertex\n"
       << el::base::debug::StackTrace();
    throw std::runtime_error(ss.str());
    return VertexPtr();
  }

  // Disable this function, since we need to know the timestamp
  VertexPtr addVertex(const RunIdType&) { return addVertex(); }
#endif

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
  storage::LockableMessage::Ptr msg_ = nullptr;

  MapInfoMsg map_info_ = MapInfoMsg();
};

template <typename DataType>
void RCGraph::registerVertexStream(const RCGraph::RunIdType& run_id,
                                   const std::string& stream_name) {
  const auto locked_runs = runs_->sharedLocked();
  const auto& runs = locked_runs.get();
  if (runs.find(run_id) == runs.end()) {
    std::stringstream ss;
    ss << "Run " << run_id << " is not in the run map.";
    CLOG(ERROR, "pose_graph") << ss.str();
    std::runtime_error{ss.str()};
  }
  runs.at(run_id)->registerVertexStream<DataType>(stream_name);
}

}  // namespace pose_graph
}  // namespace vtr