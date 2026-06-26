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
                 const CallbackPtr& callback, const bool read_only)
    : GraphType(callback),
      file_path_(file_path),
      name2accessor_map_(std::make_shared<LockableName2AccessorMap>(
          fs::path{file_path} / "data", Name2AccessorMapBase())), read_only_(read_only) {
  if (load && fs::exists(fs::path(file_path_) / "index")) {
    CLOG(INFO, "pose_graph") << "Loading pose graph from " << file_path;
    loadVertices();
    loadEdges();
    loadGraphIndex();
    buildSimpleGraph();
  } else {
    if (read_only) {
      CLOG(ERROR, "pose_graph") << "Requested pose graph as read only but requested to make a new one!";
      throw std::runtime_error("Requested making a new read-only pose graph");
    }
    CLOG(INFO, "pose_graph") << "Creating a new pose graph.";
    if (fs::exists(file_path_)) fs::remove_all(file_path_);
    auto data = std::make_shared<GraphMsg>();
    msg_ = std::make_shared<storage::LockableMessage<GraphMsg>>(data);
  }
}

void RCGraph::save() {
  std::unique_lock lock(mutex_);
  if(read_only_) {
    CLOG(ERROR, "pose_graph") << "Tried to write to a read only graph!";
    return;
  }
  CLOG(INFO, "pose_graph") << "Saving pose graph";
  saveVertices();
  // Flush any edges that saveEdgesLive() held back (always the last edge in the
  // queue, which is withheld until the next vertex arrives to confirm it is complete).
  if (!edges_to_write_.empty()) {
    EdgeMsgAccessor accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge"};
    while (!edges_to_write_.empty()) {
      accessor.write(edges_to_write_.front()->serialize());
      edges_to_write_.pop();
    }
  } else {
    saveEdges();
  }
  CLOG(INFO, "pose_graph") << "Saving pose graph - DONE!";
}

void RCGraph::saveLive() {
  std::unique_lock lock(mutex_);
  if(read_only_) { 
    CLOG(ERROR, "pose_graph") << "Tried to write to a read only graph!";
    return;
  }
  CLOG(INFO, "pose_graph") << "Saving live pose graph";
  // saveGraphIndex();
  saveVerticesLive();
  saveEdgesLive();
  CLOG(INFO, "pose_graph") << "Saving live pose graph - DONE!";
}

auto RCGraph::addVertex(const Timestamp& time) -> VertexPtr {
  // OLD:   return GraphType::addVertex(time, name2accessor_map_);
  auto vertex = GraphType::addVertex(time, name2accessor_map_);
  vertices_to_write_.push(vertex);
  if (curr_minor_id_ == 0) saveGraphIndex();
  return vertex;
}

auto RCGraph::addEdge(const VertexId& from, const VertexId& to,
                      const EdgeType& type, const EdgeMode& mode,
                      const EdgeTransform& T_to_from) -> EdgePtr {
  auto edge = GraphType::addEdge(from, to, type, mode, T_to_from);  // call base
  edges_to_write_.push(edge);
  return edge;
}

void RCGraph::loadGraphIndex() {
  GraphMsgAccessor accessor{fs::path{file_path_}, "index", "vtr_pose_graph_msgs/msg/Graph", read_only_};
  msg_ = accessor.readAtIndex(1);
  if (!msg_) {
    std::string err{"Graph index message does not exist."};
    return;
    // CLOG(ERROR, "pose_graph") << err;
    // throw std::runtime_error{err};
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

  VertexMsgAccessor accessor{fs::path{file_path_},  "vertices", "vtr_pose_graph_msgs/msg/Vertex", read_only_};
  for (int index = 1;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;

    auto vertex_msg = msg->locked().get().getData();
    auto vertex = RCVertex::MakeShared(vertex_msg, name2accessor_map_, msg);
    if (vertex->vertexTime() == 0) {
      topology_vertices_[vertex->id()] = index;
      CLOG(DEBUG, "pose_graph") << "Live: Topology Vertex inserted, len " << topology_vertices_.size();
    }
    vertices_.insert(std::make_pair(vertex->id(), vertex));
    CLOG(DEBUG, "pose_graph") << "- loaded vertex " << *vertex;
    lastVertexIdx_ = index;
  }
}

void RCGraph::loadEdges() {
  CLOG(DEBUG, "pose_graph") << "Loading edges from disk";

  EdgeMsgAccessor accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge", read_only_};
  for (int index = 1;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;

    auto edge_msg = msg->locked().get().getData();
    auto edge = RCEdge::MakeShared(edge_msg, msg);
    const auto& eid = edge->id();
    if (vertices_.find(eid.id1()) == vertices_.end() ||
        vertices_.find(eid.id2()) == vertices_.end()) {
      CLOG(WARNING, "pose_graph") << "Skipping dangling edge " << eid;
      continue;
    }
    if (edge_msg.mode.mode == vtr_pose_graph_msgs::msg::EdgeMode::UNKNOWN) {
      topology_edges_[edge->id()] = index;
      CLOG(DEBUG, "pose_graph") << "Topology Edge inserted, len " << topology_edges_.size();
    }
    edges_.insert(std::make_pair(eid, edge));
    CLOG(DEBUG, "pose_graph") << " - loaded edge " << *edge;
  }
}

void RCGraph::loadVerticesLive() {
  CLOG(DEBUG, "pose_graph") << "Live Loading vertices from disk";
  VertexMsgAccessor accessor{fs::path{file_path_},  "vertices", "vtr_pose_graph_msgs/msg/Vertex", true};

  int index = lastVertexIdx_;
  for (;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;

    auto vertex_msg = msg->locked().get().getData();
    auto vertex = RCVertex::MakeShared(vertex_msg, name2accessor_map_, msg);
    CLOG(DEBUG, "pose_graph") << "loadVerticesLive: (vertex, time): (" << vertex->id() << ", " << vertex->vertexTime() << ")";
    if (vertex->vertexTime() == 0) {
      topology_vertices_[vertex->id()] = index;
      CLOG(DEBUG, "pose_graph") << "Live: Topology Vertex inserted, len " << topology_vertices_.size();
    }
    vertices_.insert(std::make_pair(vertex->id(), vertex)); //ANTHONY
    // CLOG(DEBUG, "pose_graph") << "- live loaded vertex " << *vertex;
  }
  lastVertexIdx_ = index;
}

void RCGraph::loadEdgesLive() {
  CLOG(DEBUG, "pose_graph") << "Live Loading edges from disk";
  EdgeMsgAccessor accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge", true};

  int index = lastEdgeIdx_;
  for (;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;
    auto edge_msg = msg->locked().get().getData();
    auto edge = RCEdge::MakeShared(edge_msg, msg);
    const auto& eid = edge->id();
    if (vertices_.find(eid.id1()) == vertices_.end() ||
        vertices_.find(eid.id2()) == vertices_.end()) {
      CLOG(WARNING, "pose_graph") << "Skipping dangling edge " << eid;
      continue;
    }
    if (edge_msg.mode.mode == vtr_pose_graph_msgs::msg::EdgeMode::UNKNOWN) {
      topology_edges_[edge->id()] = index;
      CLOG(DEBUG, "pose_graph") << "Live: Topology Edge inserted, len " << topology_vertices_.size();
    }
    edges_.insert(std::make_pair(eid, edge)); //ANTHONY
    CLOG(DEBUG, "pose_graph") << " - loaded edge " << *edge;
  }
  lastEdgeIdx_ = index;
}

void RCGraph::populateLive() {
  CLOG(DEBUG, "pose_graph") << "Monitoring topology-only edges";
  CLOG(DEBUG, "pose_graph") << "# topology edges, vertices: " << topology_edges_.size();
  // Monitor edges that were topology, overwrite internal rep. of edges/vertices
  EdgeMsgAccessor edge_accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge", true};
  VertexMsgAccessor vtx_accessor{fs::path{file_path_}, "vertices", "vtr_pose_graph_msgs/msg/Vertex", true};

  { 
    std::unique_lock lock(mutex_);

    //populate vertices
    for (auto vtx_it = topology_vertices_.begin(); vtx_it != topology_vertices_.end();) {
      const auto msg = vtx_accessor.readAtIndex(vtx_it->second);
      if (!msg) {break; }

      auto updated_vtx_msg = msg->locked().get().getData();
      auto new_vertex = RCVertex::MakeShared(updated_vtx_msg, name2accessor_map_, msg);
      
      if (new_vertex->vertexTime() != 0 ) {
        const auto& vid = new_vertex->id();
        vertices_[vid] = new_vertex;
        CLOG(DEBUG, "pose_graph") << "populateLive: overwrote vertex " << vid << " with real timestamp: " << new_vertex->vertexTime();
        vtx_it = topology_vertices_.erase(vtx_it);
      } else {
        ++vtx_it;
      }
    }
  }
    
  std::vector<RCEdge::Ptr> edges_to_publish;
  {
    std::unique_lock lock(mutex_);

    // populate edges
    for (auto e_it = topology_edges_.begin(); e_it != topology_edges_.end();) {
      const auto msg = edge_accessor.readAtIndex(e_it->second);
      if (!msg) break;
      auto updated_edge_msg = msg->locked().get().getData();
      if (updated_edge_msg.mode.mode == vtr_pose_graph_msgs::msg::EdgeMode::MANUAL) {
        auto new_edge = RCEdge::MakeShared(updated_edge_msg, msg);
        const auto& eid = new_edge->id();

        CLOG(DEBUG, "pose_graph") << "populateEdgesLive: overwrote edge" << eid;
        edges_.at(eid) = new_edge;
        edges_to_publish.push_back(new_edge);
        e_it = topology_edges_.erase(e_it);
      } else {
        ++e_it;
      }
    }
  }

  for (const auto& edge : edges_to_publish) {
    callback_->publishUpdate(edge);
  }
}

void RCGraph::loadLive() {
  CLOG(DEBUG, "pose_graph") << "loadLive";
  loadVerticesLive();
  loadEdgesLive();
  populateLive();
  return;
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
  if(read_only_) { 
    CLOG(ERROR, "pose_graph") << "Tried to write to a read only graph!";
    return;
  }

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
  CLOG(DEBUG, "pose_graph") << "Saving graph index";
  accessor.write(msg_);
}

void RCGraph::saveVertices() {
  if(read_only_) { 
    CLOG(ERROR, "pose_graph") << "Tried to write to a read only graph!";
    return;
  }

  // save any unsaved data first
  CLOG(DEBUG, "pose_graph") << "Saving vertices to disk";
  for (auto iter = vertices_.begin(); iter != vertices_.end(); ++iter)
    iter->second->unload();
  VertexMsgAccessor accessor{fs::path{file_path_}, "vertices", "vtr_pose_graph_msgs/msg/Vertex"};
  for (auto it = vertices_.begin(); it != vertices_.end(); ++it)
    accessor.write(it->second->serialize());
}

void RCGraph::saveVerticesLive() {
  if(read_only_) { 
    CLOG(ERROR, "pose_graph") << "Tried to write to a read only graph!";
    return;
  }

  // save any unsaved data first
  CLOG(DEBUG, "pose_graph") << "Saving vertices to disk, Vertex Q len : " << vertices_to_write_.size();
  VertexMsgAccessor accessor{fs::path{file_path_}, "vertices", "vtr_pose_graph_msgs/msg/Vertex"};
  while (vertices_to_write_.size() > 1){
    auto vertex = vertices_to_write_.front();
    vertices_to_write_.pop();
    vertex->unload();
    accessor.write(vertex->serialize());
    lastVertexIdx_++;
  }  
}

void RCGraph::saveEdges() {
  if(read_only_) { 
    CLOG(ERROR, "pose_graph") << "Tried to write to a read only graph!";
    return;
  }

  CLOG(DEBUG, "pose_graph") << "Saving edges to disk";
  EdgeMsgAccessor accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge"};
  for (auto it = edges_.begin(); it != edges_.end(); ++it)
    accessor.write(it->second->serialize());
}

void RCGraph::saveEdgesLive() {
  if(read_only_) { 
    CLOG(ERROR, "pose_graph") << "Tried to write to a read only graph!";
    return;
  }

  // save any unsaved data first
  CLOG(DEBUG, "pose_graph") << "Saving Edges to disk, Edge Q len : " << edges_to_write_.size();
  EdgeMsgAccessor accessor{fs::path{file_path_}, "edges", "vtr_pose_graph_msgs/msg/Edge"};
  // save all but current edge
  while (edges_to_write_.size() > 1){
    auto edge = edges_to_write_.front();
    edges_to_write_.pop();
    accessor.write(edge->serialize());
    lastEdgeIdx_++;
  }  
}

}  // namespace pose_graph
}  // namespace vtr
