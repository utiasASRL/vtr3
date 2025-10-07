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
  // Cursor: If 'load' is true and an index exists on disk, load a persisted
  // Cursor: graph (index + vertices + edges) and then rebuild the lightweight
  // Cursor: SimpleGraph for topological operations. Otherwise, initialize a new
  // Cursor: graph and prepare an empty lockable Graph ROS message.
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
  // Cursor: Serialize the current in-memory graph back to disk.
  // Cursor: Take an exclusive lock to guard against concurrent mutation and
  // Cursor: write index, vertices, then edges.
  std::unique_lock lock(mutex_);
  CLOG(INFO, "pose_graph") << "Saving pose graph";
  saveGraphIndex();
  saveVertices();
  saveEdges();
  CLOG(INFO, "pose_graph") << "Saving pose graph - DONE!";
}

auto RCGraph::addVertex(const Timestamp& time) -> VertexPtr {
  // Cursor: Delegate to base to create a vertex that shares the accessor map.
  // Cursor: The shared accessor map allows all vertices/edges to lazily access
  // Cursor: their on-disk bubbles under <file_path>/data/.
  return GraphType::addVertex(time, name2accessor_map_);
}

void RCGraph::loadGraphIndex() {
  // Cursor: Read the single graph-level metadata message (index) from disk
  // Cursor: and populate IDs and map_info.
  GraphMsgAccessor accessor{fs::path{file_path_}, "index", "vtr_pose_graph_msgs/msg/Graph"};
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
  // Cursor: Sequentially read vertex messages until no more entries exist;
  // Cursor: for each, construct an RCVertex that shares the accessor map and
  // Cursor: store it in the in-memory vertex map.
  CLOG(DEBUG, "pose_graph") << "Loading vertices from disk";

  VertexMsgAccessor accessor{fs::path{file_path_},  "vertices", "vtr_pose_graph_msgs/msg/Vertex"};
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
  // Cursor: Sequentially read edge messages until no more entries exist;
  // Cursor: for each, construct an RCEdge bound to the lockable message and
  // Cursor: store it in the in-memory edge map.
  CLOG(DEBUG, "pose_graph") << "Loading edges from disk";

  EdgeMsgAccessor accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge"};
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
  // Cursor: Populate the internal SimpleGraph with the set of vertex IDs and
  // Cursor: edge IDs to enable fast topological queries independent of heavy
  // Cursor: per-vertex/edge data.
  // First add all vertices to the simple graph
  for (auto it = vertices_.begin(); it != vertices_.end(); ++it)
    graph_.addVertex(it->first);
  // Add all edges to the simple graph
  for (auto it = edges_.begin(); it != edges_.end(); ++it)
    graph_.addEdge(it->first);
}

void RCGraph::saveGraphIndex() {
  // Cursor: Write the graph-level metadata (IDs and map_info) back to disk as
  // Cursor: a single Graph message under <file_path>/index.
  GraphMsg data;
  data.curr_major_id = curr_major_id_;
  data.curr_minor_id = curr_minor_id_;
  data.map_info = getMapInfo();

  CLOG(DEBUG, "pose_graph") << "Saving pose graph to disk";
  CLOG(DEBUG, "pose_graph") << "- graph curr major id: " << data.curr_major_id;
  CLOG(DEBUG, "pose_graph") << "- graph curr minor id: " << data.curr_minor_id;
  CLOG(DEBUG, "pose_graph") << "- graph map info set: " << data.map_info.set;

  msg_->locked().get().setData(data);

  GraphMsgAccessor accessor{fs::path{file_path_}, "index", "vtr_pose_graph_msgs/msg/Graph"};
  accessor.write(msg_);
}

void RCGraph::saveVertices() {
  // Cursor: Ensure any lazily-held vertex data is flushed/unloaded, then write
  // Cursor: each vertex's serialized message to <file_path>/vertices.
  // save any unsaved data first
  CLOG(DEBUG, "pose_graph") << "Saving vertices to disk";
  for (auto iter = vertices_.begin(); iter != vertices_.end(); ++iter)
    iter->second->unload();
  VertexMsgAccessor accessor{fs::path{file_path_}, "vertices", "vtr_pose_graph_msgs/msg/Vertex"};
  for (auto it = vertices_.begin(); it != vertices_.end(); ++it)
    accessor.write(it->second->serialize());
}

void RCGraph::saveEdges() {
  // Cursor: Write each edge's serialized message to <file_path>/edges.
  CLOG(DEBUG, "pose_graph") << "Saving edges to disk";
  EdgeMsgAccessor accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge"};
  for (auto it = edges_.begin(); it != edges_.end(); ++it)
    accessor.write(it->second->serialize());
}

}  // namespace pose_graph
}  // namespace vtr
