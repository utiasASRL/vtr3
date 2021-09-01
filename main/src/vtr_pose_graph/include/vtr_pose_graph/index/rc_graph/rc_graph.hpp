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
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/utils/container_tools.hpp>
#include <vtr_messages/msg/graph_map_info.hpp>
#include <vtr_messages/msg/graph_run_list.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/graph.hpp>
#include <vtr_pose_graph/index/rc_graph/persistent.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph_base.hpp>
#include <vtr_pose_graph/index/rc_graph/types.hpp>

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
  using Base::id_;
  using Base::runs_;
  using Base::vertices_;

  using RunFilter = std::unordered_set<RunIdType>;

  using GraphIndexMsg = vtr_messages::msg::GraphRunList;
  using MapInfoMsg = vtr_messages::msg::GraphMapInfo;

  PTR_TYPEDEFS(RCGraph);

  /** \brief Pseudo constructor for making shared pointers */
  static Ptr MakeShared() { return Ptr(new RCGraph()); }
  static Ptr MakeShared(const std::string& filePath, const IdType& id) {
    return Ptr(new RCGraph(filePath, id));
  }
  static Ptr MakeShared(const std::string& filePath) {
    return Ptr(new RCGraph(filePath));
  }
  static Ptr LoadOrCreate(const std::string& filePath,
                          const IdType& id = IdType(0));

  /** \brief Default constructor */
  RCGraph() : Base(), RCGraphBase(), GraphType(), filePath_(""), msg_() {
    msg_.last_run = uint32_t(-1);
  }
  /** \brief Construct an empty graph with an id and save location */
  RCGraph(const std::string& filePath, const IdType& id)
      : Base(id), RCGraphBase(id), GraphType(id), filePath_(filePath), msg_() {
    msg_.graph_id = this->id_;
    msg_.last_run = uint32_t(-1);
    saveIndex();
  }

  /** \brief Construct an graph, pointing to an index file */
  RCGraph(const std::string& filePath)
      : Base(), RCGraphBase(), GraphType(), filePath_(filePath), msg_() {
    msg_.graph_id = this->id_;
    msg_.last_run = uint32_t(-1);
  }

  /// \note Yuchen: we used to allow copying and moving, but I don't think it is
  /// needed or even safe to do so.
  RCGraph(const RCGraph&) = delete;
  RCGraph(RCGraph&& other) = delete;
  RCGraph& operator=(const RCGraph&) = delete;
  RCGraph& operator=(RCGraph&& other) = delete;

  /** \brief Return a blank vertex(current run) with the next available Id */
  virtual VertexPtr addVertex(const vtr_messages::msg::TimeStamp& time) {
    return addVertex(time, currentRun_->id());
  }

  /** \brief Return a blank vertex with the next available Id */
  virtual VertexPtr addVertex(const vtr_messages::msg::TimeStamp& time,
                              const RunIdType& runId);

  /** \brief Deep load all levels of index data */
  void load(const RunFilter& r = RunFilter());
  /** \brief Load the top-level index from file */
  void loadIndex();
  /** \brief Deep load runs and their vertex/edge data */
  void loadRuns(const RunFilter& r = RunFilter());
  /** \brief Load the indexes of each run to inspect descriptions */
  void loadRunIndexes(const RunFilter& r = RunFilter());
#if 0
  /** \brief Save all modifications to temporary files */
  void saveWorking();
  /** \brief Save the top-level index to a working file */
  void saveWorkingIndex();
  /** \brief Save all modified runs to temporary files */
  void saveWorkingRuns();
#endif
  /** \brief Save everything to file */
  void save(bool force = false);
  /** \brief Save the top-level index to file */
  void saveIndex();
  /** \brief Save all modified runs to file */
  void saveRuns(bool force = false);

  /** \brief Get the file path of the graph index */
  std::string filePath() const { return filePath_; }

  /**
   * \brief Add a new run an increment the run id
   * \details This function is disabled for RCGraphs....
   */
  RunIdType addRun() override {
    std::stringstream ss;
    ss << "addRun(robotId) must be called for RCGraphs\n"
       << el::base::debug::StackTrace();
    throw std::runtime_error(ss.str());
    return RunIdType(-1);
  }

  /** \brief Add a new run an increment the run id */
  virtual RunIdType addRun(IdType robotId, bool ephemeral = false,
                           bool extend = false, bool dosave = true);

  /** \brief Removes any temporary runs, if they exist */
  void removeEphemeralRuns();

  /** \brief Check if a specific run has a given stream */
  bool hasVertexStream(RunIdType rid, const std::string& stream_name) const {
    if (runs_ == nullptr) return false;
    const auto& run = runs_->find(rid);
    if (run == runs_->end()) return false;
    return run->second->hasVertexStream(stream_name);
  }

  /**
   * \brief registers a stream to a run.
   * \todo (yuchen) the points_to_data flag seems useless
   */
  template <typename MessageType>
  void registerVertexStream(const RunIdType& run_id,
                            const std::string& stream_name,
                            bool points_to_data = true,
                            const RegisterMode& mode = RegisterMode::Create);
#if 0
  /** \brief Ensure correct vertex indices for a data stream */
  void reindexStream(const RunIdType& run_id, const std::string& stream_name,
                     const WindowType& wType = WindowType::Before);

  /** \brief Raw stream write access for data that isn't vertex-indexed */
  inline const SerializerPtr& writeStream(const RunIdType& run_id,
                                          const std::string& streamName) const {
    if (runs_ != nullptr && runs_->find(run_id) != runs_->end()) {
      return runs_->at(run_id)->writeStream(streamName);
    } else {
      LOG(ERROR) << "[RCGraphBase::readStream] Run " << run_id
                 << " was not in the run map.";
      throw std::runtime_error(
          "[RCGraph::writeStream] Attempted to access non-existent run");
    }
  }
#endif

  /** \brief Get the map display calibration */
  /// \todo yuchen is this safe?
  const MapInfoMsg& mapInfo() const { return msg_.map; }

  /** \brief Get the map display calibration */
  /// \todo yuchen is this safe?
  MapInfoMsg& mutableMapInfo() { return msg_.map; }

  /** \brief Determine if a display map has been set for this graph */
  bool hasMap() const { return msg_.map.set; }

  /** \brief Set the map display calibration */
  void setMapInfo(const MapInfoMsg& map) {
    msg_.map = map;
    msg_.map.set = true;  // manually set to true in case we forget it in argin
  }

  /** \brief Remove map information from a graph (USE CAREFULLY) */
  void clearMap() { msg_.map.set = false; }

  /**
   * \brief Removes any empty runs and associated folders from the graph.
   *        USE CAREFULLY, and only when you are shutting down the program.
   */
  void halt();

 protected:
  // Disable this function, since we need to know the timestamp
  VertexPtr addVertex() override {
    std::stringstream ss;
    ss << "Must provide timestamps for RCVertex\n"
       << el::base::debug::StackTrace();
    throw std::runtime_error(ss.str());
    return VertexPtr();
  }

  // Disable this function, since we need to know the timestamp
  VertexPtr addVertex(const RunIdType&) override { return addVertex(); }

  /**
   * \brief Builds the simple graph using loaded vertices and edges, and add
   * neighbor information to vertices_.
   */
  void buildSimpleGraphAndAddVertexNeighbors();

  /** \brief Build map from persistent ids to existing vertex ids */
  void buildPersistentMap();

  std::string filePath_;

  /** \brief Ros message containing necessary information for a list of runs. */
  GraphIndexMsg msg_;
};

#if 0
// TODO: Find a way to make explicit instantiation work in debug mode
#if !defined(RCGRAPH_NO_EXTERN) && defined(NDEBUG)
extern template class Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;

typedef Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>> __GraphRC;
EVAL_TYPED_DECLARE_EXTERN(double, __GraphRC)
EVAL_TYPED_DECLARE_EXTERN(bool, __GraphRC)

EVAL_TYPED_DECLARE_EXTERN(double, RCGraph)
EVAL_TYPED_DECLARE_EXTERN(bool, RCGraph)
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr

#include "rc_graph.inl"
