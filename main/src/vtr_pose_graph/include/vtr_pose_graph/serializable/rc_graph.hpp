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
#include "vtr_pose_graph/serializable/rc_edge.hpp"
#include "vtr_pose_graph/serializable/rc_vertex.hpp"

#include "vtr_pose_graph_msgs/msg/graph.hpp"
#include "vtr_pose_graph_msgs/msg/map_info.hpp"
#include "vtr_pose_graph_msgs/msg/timestamp.hpp"

namespace vtr {
namespace pose_graph {

// Virtual inheritance is necessary to ensure that relevant methods in GraphBase
// stay overridden
class RCGraph : public Graph<RCVertex, RCEdge> {
 public:
  PTR_TYPEDEFS(RCGraph);

  using GraphType = Graph<RCVertex, RCEdge>;
  using Base = GraphType::Base;

  using Vertex = typename Base::Vertex;
  using VertexPtr = typename Base::VertexPtr;

  using Edge = typename Base::Edge;
  using EdgePtr = typename Base::EdgePtr;

  using Callback = GraphCallbackInterface<RCVertex, RCEdge>;
  using CallbackPtr = typename Callback::Ptr;

  using GraphMsg = vtr_pose_graph_msgs::msg::Graph;
  using MapInfoMsg = vtr_pose_graph_msgs::msg::MapInfo;

  /// graph and data serialization
  using GraphMsgAccessor = storage::DataStreamAccessor<GraphMsg>;
  using VertexMsgAccessor = storage::DataStreamAccessor<RCVertex::VertexMsg>;
  using EdgeMsgAccessor = storage::DataStreamAccessor<RCEdge::EdgeMsg>;

  using DataAccessor = std::shared_ptr<storage::DataStreamAccessorBase>;
  using Name2AccessorMapBase = std::unordered_map<std::string, DataAccessor>;
  using Name2AccessorMap = std::pair<const std::string, Name2AccessorMapBase>;
  using LockableName2AccessorMap = common::SharedLockable<Name2AccessorMap>;
  using Name2AccessorMapPtr = std::shared_ptr<LockableName2AccessorMap>;

  /** \brief Pseudo constructor for making shared pointers */
  static Ptr MakeShared(
      const std::string& file_path, const bool load = true,
      const CallbackPtr& callback = std::make_shared<Callback>()) {
    return std::make_shared<RCGraph>(file_path, load, callback);
  }

  /** \brief Construct an empty graph with an id and save location */
  RCGraph(const std::string& file_path, const bool load = true,
          const CallbackPtr& callback = std::make_shared<Callback>());

  virtual ~RCGraph() { save(); }

  void save();

  /** \brief Return a blank vertex with the next available Id */
  VertexPtr addVertex(const Timestamp& time);

  /** \brief Get the map display calibration */
  MapInfoMsg getMapInfo() const {
    std::shared_lock lock(map_info_mutex_);
    return map_info_;
  }

  /** \brief Set the map display calibration */
  void setMapInfo(const MapInfoMsg& map_info) {
    std::unique_lock lock(map_info_mutex_);
    map_info_ = map_info;
  }

  /** \brief Get the file path of the graph index */
  std::string filePath() const { return file_path_; }

  /** \brief Writes a message to disk directly, without associated vertex.*/
  template <typename DataType>
  void write(const std::string& stream_name, const std::string& stream_type,
             const typename storage::LockableMessage<DataType>::Ptr& message);

 private:
  /** \brief Helper methods for loading from disk */
  void loadGraphIndex();
  void loadVertices();
  void loadEdges();
  void buildSimpleGraph();

  /** \brief Helper methods for saving to disk */
  void saveGraphIndex();
  void saveVertices();
  void saveEdges();

 private:
  using Base::mutex_;

  using Base::graph_;

  using Base::vertices_;

  using Base::edges_;

  const std::string file_path_;

  const Name2AccessorMapPtr name2accessor_map_;

  /** \brief Ros message containing necessary information for a list of runs. */
  storage::LockableMessage<GraphMsg>::Ptr msg_ = nullptr;

  mutable std::shared_mutex map_info_mutex_;
  MapInfoMsg map_info_ = MapInfoMsg();
};

template <typename DataType>
void RCGraph::write(
    const std::string& stream_name, const std::string& stream_type,
    const typename storage::LockableMessage<DataType>::Ptr& message) {
  // check if exists (with a shared lock so that it does not block other reads)
  {
    const auto name2accessor_map_locked = name2accessor_map_->sharedLocked();
    const auto& name2accessor_map_ref = name2accessor_map_locked.get();

    const auto accessor_itr = name2accessor_map_ref.second.find(stream_name);
    if (accessor_itr != name2accessor_map_ref.second.end()) {
      std::dynamic_pointer_cast<storage::DataStreamAccessor<DataType>>(
          accessor_itr->second)
          ->write(message);
      return;
    }
  }

  // perform insertion to the map
  const auto name2accessor_map_locked = name2accessor_map_->locked();
  auto& name2accessor_map_ref = name2accessor_map_locked.get();

  const auto accessor_itr = name2accessor_map_ref.second.try_emplace(
      stream_name, std::make_shared<storage::DataStreamAccessor<DataType>>(
                       name2accessor_map_ref.first, stream_name, stream_type));
  std::dynamic_pointer_cast<storage::DataStreamAccessor<DataType>>(
      accessor_itr.first->second)
      ->write(message);
}

}  // namespace pose_graph
}  // namespace vtr