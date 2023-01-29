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
 * \file rc_graph.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_pose_graph/serializable/rc_graph.hpp"

#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {

RCGraph::RCGraph(const std::string& file_path, const bool load,
                 const CallbackPtr& callback)
    : GraphType(callback),
      file_path_(file_path),
      name2accessor_map_(std::make_shared<LockableName2AccessorMap>(
          fs::path{file_path} / "data", Name2AccessorMapBase())) {
  if (load && fs::exists(fs::path(file_path_) / "index")) {
    CLOG(INFO, "pose_graph") << "Loading pose graph from " << file_path;
    loadGraphIndex();
    loadVertices();
    loadEdges();
    buildSimpleGraph();
  } else {
    CLOG(INFO, "pose_graph") << "Creating a new pose graph.";
    if (fs::exists(file_path_)) fs::remove_all(file_path_);
    auto data = std::make_shared<GraphMsg>();
    msg_ = std::make_shared<storage::LockableMessage<GraphMsg>>(data);
  }
}

void RCGraph::save() {
  std::unique_lock lock(mutex_);
  CLOG(INFO, "pose_graph") << "Saving pose graph";
  saveGraphIndex();
  saveVertices();
  saveEdges();
  CLOG(INFO, "pose_graph") << "Saving pose graph - DONE!";
}

auto RCGraph::addVertex(const Timestamp& time) -> VertexPtr {
  return GraphType::addVertex(time, name2accessor_map_);
}

void RCGraph::loadGraphIndex() {
  GraphMsgAccessor accessor{fs::path{file_path_} / "index"};
  msg_ = accessor.readAtIndex(1);
  if (!msg_) {
    std::string err{"Graph index message does not exist."};
    CLOG(ERROR, "pose_graph") << err;
    throw std::runtime_error{err};
  }
  const auto data = msg_->locked().get().getData();

  CLOG(DEBUG, "pose_graph") << "Loading pose graph index from disk";
  CLOG(DEBUG, "pose_graph") << "- graph curr major id: " << data.curr_major_id;
  CLOG(DEBUG, "pose_graph") << "- graph curr minor id: " << data.curr_minor_id;
  CLOG(DEBUG, "pose_graph") << "- graph map info set: " << data.map_info.set;

  curr_major_id_ = data.curr_major_id;
  curr_minor_id_ = data.curr_minor_id;

  map_info_ = data.map_info;
}

void RCGraph::loadVertices() {
  CLOG(DEBUG, "pose_graph") << "Loading vertices from disk";

  VertexMsgAccessor accessor{fs::path{file_path_} / "vertices"};
  for (int index = 1;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;

    auto vertex_msg = msg->locked().get().getData();
    auto vertex = RCVertex::MakeShared(vertex_msg, name2accessor_map_, msg);
    vertices_.insert(std::make_pair(vertex->id(), vertex));
    CLOG(DEBUG, "pose_graph") << "- loaded vertex " << *vertex;
  }
}

void RCGraph::loadEdges() {
  CLOG(DEBUG, "pose_graph") << "Loading edges from disk";

  EdgeMsgAccessor accessor{fs::path{file_path_} / "edges"};
  for (int index = 1;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;

    auto edge_msg = msg->locked().get().getData();
    auto edge = RCEdge::MakeShared(edge_msg, msg);
    edges_.insert(std::make_pair(edge->id(), edge));
    CLOG(DEBUG, "pose_graph") << " - loaded edge " << *edge;
  }
}

void RCGraph::buildSimpleGraph() {
  // First add all vertices to the simple graph
  for (auto it = vertices_.begin(); it != vertices_.end(); ++it)
    graph_.addVertex(it->first);
  // Add all edges to the simple graph
  for (auto it = edges_.begin(); it != edges_.end(); ++it)
    graph_.addEdge(it->first);
}

void RCGraph::saveGraphIndex() {
  GraphMsg data;
  data.curr_major_id = curr_major_id_;
  data.curr_minor_id = curr_minor_id_;
  data.map_info = getMapInfo();

  CLOG(DEBUG, "pose_graph") << "Saving pose graph to disk";
  CLOG(DEBUG, "pose_graph") << "- graph curr major id: " << data.curr_major_id;
  CLOG(DEBUG, "pose_graph") << "- graph curr minor id: " << data.curr_minor_id;
  CLOG(DEBUG, "pose_graph") << "- graph map info set: " << data.map_info.set;

  msg_->locked().get().setData(data);

  GraphMsgAccessor accessor{fs::path{file_path_} / "index", "index", "vtr_pose_graph_msgs/msg/Graph"};
  accessor.write(msg_);
}

void RCGraph::saveVertices() {
  // save any unsaved data first
  CLOG(DEBUG, "pose_graph") << "Saving any unsaved data in cache";
  for (auto iter = vertices_.begin(); iter != vertices_.end(); ++iter)
    iter->second->unload();

  CLOG(DEBUG, "pose_graph") << "Saving vertices to disk";
  VertexMsgAccessor accessor{fs::path{file_path_} / "vertices", "vertices", "vtr_pose_graph_msgs/msg/Vertex"};
  for (auto it = vertices_.begin(); it != vertices_.end(); ++it)
    accessor.write(it->second->serialize());
}

void RCGraph::saveEdges() {
  CLOG(DEBUG, "pose_graph") << "Saving edges to disk";
  EdgeMsgAccessor accessor{fs::path{file_path_} / "edges", "edges", "vtr_pose_graph_msgs/msg/Edge"};
  for (auto it = edges_.begin(); it != edges_.end(); ++it)
    accessor.write(it->second->serialize());
}

}  // namespace pose_graph
}  // namespace vtr
