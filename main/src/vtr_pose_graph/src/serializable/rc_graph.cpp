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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <filesystem>
#include <iomanip>  // \todo (yuchen) needed for setw/setfill

#include <vtr_pose_graph/serializable/rc_graph.hpp>

#include <vtr_pose_graph_msgs/msg/persistent_id.hpp>

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {

namespace {

inline std::string getRunDir(const uint32_t id) {
  std::stringstream ss;
  ss << "run_" << std::setfill('0') << std::setw(6) << id;
  return ss.str();
}
}  // namespace

RCGraph::RCGraph(const std::string& file_path, const bool load)
    : Base(), RCGraphBase(), GraphType(), file_path_(file_path) {
  if (load && fs::exists(fs::path(file_path_) / "graph_index")) {
    CLOG(INFO, "pose_graph") << "Loading pose graph from " << file_path;
    loadGraphIndex();
    loadRuns();
    buildSimpleGraphAndAddVertexNeighbors();
    buildPersistentMap();
  } else {
    CLOG(INFO, "pose_graph") << "Creating a new pose graph.";
    if (fs::exists(file_path_)) fs::remove_all(file_path_);
    auto data = std::make_shared<GraphMsg>();
    msg_ = std::make_shared<storage::LockableMessage<GraphMsg>>(data);
  }
}

void RCGraph::save() {
  LockGuard lck(mtx_);
  CLOG(INFO, "pose_graph") << "Saving pose graph";

  // save any unsaved data first
  {
    CLOG(INFO, "pose_graph") << "Saving any unsaved data in cache";
    const auto locked_vertices = vertices_->sharedLocked();
    const auto& vertices = locked_vertices.get();
    for (auto iter = vertices.begin(); iter != vertices.end(); ++iter)
      iter->second->unload();
  }

  saveGraphIndex();
  saveRuns();

  CLOG(INFO, "pose_graph") << "Saving pose graph - done!";
}

RCGraph::RunIdType RCGraph::addRun() {
  LockGuard lck(mtx_);
  auto new_run_id = last_run_id_ + 1;
  const auto file_path = fs::path{file_path_} / getRunDir(new_run_id);
  return GraphType::addRun(file_path);
}

auto RCGraph::addVertex(const Timestamp& time, const RunIdType& run_id)
    -> VertexPtr {
  LockGuard lock(mtx_);

  VertexPtr vertex = nullptr;
  {
    std::unique_lock graph_lock(simple_graph_mutex_, std::defer_lock);
    std::unique_lock pst_map_lock(persistent_map_->mutex(), std::defer_lock);
    std::unique_lock vertices_lock(vertices_->mutex(), std::defer_lock);
    std::shared_lock runs_lock(runs_->mutex(), std::defer_lock);
    std::lock(graph_lock, vertices_lock, pst_map_lock, runs_lock);

    // Check that persistent id doesn't already exist before adding the vertex.
    const PersistentIdType persistent_id = time;
    auto result = persistent_map_->unlocked().get().emplace(
        persistent_id, VertexId::Invalid());
    // If the persistent id already exists, then throw an exception
    if (result.second == false)
      throw std::runtime_error(
          "Persistent ID already exists when trying to add vertex");

    auto& runs = runs_->unlocked().get();
    vertex = runs.at(run_id)->addVertex(time);
    vertices_->unlocked().get().insert({vertex->simpleId(), vertex});

    // update vertex id in the persistent map
    result.first->second = vertex->id();

    graph_.addVertex(vertex->simpleId());
  }

  callback_->vertexAdded(vertex);

  return vertex;
}

void RCGraph::loadGraphIndex() {
  GraphMsgAccessor accessor{fs::path{file_path_} / "graph_index"};
  msg_ = accessor.readAtIndex(1);
  if (!msg_) {
    std::string err{"Graph index message does not exist."};
    CLOG(ERROR, "pose_graph") << err;
    throw std::runtime_error{err};
  }
  const auto data = msg_->locked().get().getData();

  std::stringstream ss;
  ss << "Loading pose graph index from message" << std::endl;
  ss << "- graph last run id: " << data.last_run << std::endl;
  ss << "- graph map info set: " << data.map_info.set << std::endl;
  CLOG(DEBUG, "pose_graph") << ss.str();

  last_run_id_ = data.last_run;
  map_info_ = data.map_info;
}

void RCGraph::loadRuns(const RunFilter& run_filter) {
  RunMsgAccessor accessor{fs::path{file_path_} / "run_index"};

  CLOG(DEBUG, "pose_graph") << "Loading pose graph runs.";

  for (int index = 1;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;

    auto data = msg->locked().get().getData();
    if (!run_filter.empty() && !common::utils::contains(run_filter, data.id))
      continue;

    CLOG(DEBUG, "pose_graph") << "Loading run with id: " << data.id;
    /// \note only called in constructor so no need to lock
    auto tmp_run = RCRun::MakeShared(fs::path{file_path_} / getRunDir(data.id),
                                     data, vertices_->unlocked().get(),
                                     edges_->unlocked().get(), run_filter, msg);
    auto& runs = runs_->unlocked().get();
    runs.insert(std::make_pair(tmp_run->id(), tmp_run));
    current_run_ = runs.at(tmp_run->id());
  }
}

void RCGraph::saveGraphIndex() {
  GraphMsg data;
  data.last_run = last_run_id_ == static_cast<uint32_t>(-1) ? -1 : last_run_id_;
  data.map_info = map_info_;

  std::stringstream ss;
  ss << "Saving graph index to message" << std::endl;
  ss << "- graph last run id: " << data.last_run << std::endl;
  ss << "- graph map info set: " << data.map_info.set << std::endl;
  CLOG(DEBUG, "pose_graph") << ss.str();

  msg_->locked().get().setData(data);

  GraphMsgAccessor accessor{fs::path{file_path_} / "graph_index"};
  accessor.write(msg_);
}

void RCGraph::saveRuns() {
  RunMsgAccessor writer{fs::path{file_path_} / "run_index"};
  CLOG(DEBUG, "pose_graph") << "Saving runs in the graph";

  const auto locked_runs = runs_->sharedLocked();
  const auto& runs = locked_runs.get();

  for (auto it = runs.begin(); it != runs.end(); ++it) {
    // Don't save empty runs
    if (it->second->numberOfVertices() == 0) {
      CLOG(WARNING, "pose_graph")
          << "Not saving run with id " << it->second->id()
          << "because the run is empty.";
      if (it->second->id() != last_run_id_) {
        std::string err{"Empty run is not the last run - this is not allowed"};
        CLOG(ERROR, "pose_graph") << err;
        throw std::runtime_error{err};
      }
      continue;
    }
    writer.write(it->second->serialize());
  }
}

void RCGraph::buildSimpleGraphAndAddVertexNeighbors() {
  LockGuard lck(mtx_);

  if (graph_.numberOfEdges() > 0) {
    std::string err{"Cannot re-link edges on an existing graph!"};
    CLOG(ERROR, "pose_graph") << err;
    throw std::runtime_error{err};
  }

  auto& vertices = vertices_->unlocked().get();
  auto& edges = edges_->unlocked().get();

  // First add all vertices to the simple graph in case there are isolated
  // vertices.
  for (auto it = vertices.begin(); it != vertices.end(); ++it)
    graph_.addVertex(it->second->simpleId());

  // Add all edges to the simple graph as well as vertices they are connected
  // to.
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    if (vertices.find(it->second->from()) != vertices.end() &&
        vertices.find(it->second->to()) != vertices.end()) {
      vertices.at(it->second->from())->addEdge(it->second->id());
      vertices.at(it->second->to())->addEdge(it->second->id());
      graph_.addEdge(it->second->from(), it->second->to());
    } else {
      CLOG(ERROR, "pose_graph")
          << " Could not link vertices " << it->second->from() << " and "
          << it->second->to();
      CLOG(ERROR, "pose_graph")
          << "Contains " << it->second->from() << " "
          << (vertices.find(it->second->from()) != vertices.end());
      CLOG(ERROR, "pose_graph")
          << "Contains " << it->second->to() << " "
          << (vertices.find(it->second->to()) != vertices.end());
      throw std::runtime_error{"Constructing simple graph failed."};
    }
  }
}

void RCGraph::buildPersistentMap() {
  /// Only called in constructor - no need to lock

  // Lock and initialize the map
  auto& map = persistent_map_->unlocked().get();
  map.clear();
  map.reserve(numberOfVertices());

  // add all the vertices to the map
  const auto& vertices = vertices_->unlocked().get();
  for (auto it = vertices.begin(); it != vertices.end(); ++it) {
    auto& vertex = it->second;
    auto vertex_id = vertex->id();
    map.emplace(vertex->persistentId(), vertex_id);
  }
}

}  // namespace pose_graph
}  // namespace vtr
