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
#if 0
#define RCGRAPH_NO_EXTERN
#endif

#include <filesystem>
#include <iomanip>  // \todo (yuchen) needed for setw/setfill

#include <vtr_messages/msg/graph_persistent_id.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {

RCGraph::Ptr RCGraph::LoadOrCreate(const std::string& filePath,
                                   const IdType& id) {
  if (fs::exists(filePath)) {
    CLOG(INFO, "pose_graph") << "Loading existing pose graph.";
    auto graph = Ptr(new RCGraph(filePath));
    graph->load();
    return graph;
  }
  return Ptr(new RCGraph(filePath, id));
}

auto RCGraph::addVertex(const vtr_messages::msg::TimeStamp& time,
                        const RunIdType& runId) -> VertexPtr {
  LockGuard lock(mtx_);

  // Check that the persistent id doesn't already exist before adding the
  // vertex.
  uint64_t stamp = time.nanoseconds_since_epoch;
  uint32_t robot = runs_->at(runId)->robotId();

  auto candidate_persistent_id = GraphPersistentIdMsg();
  candidate_persistent_id.stamp = stamp;
  candidate_persistent_id.robot = robot;

  auto locked_map = persistent_map_->locked();
  auto insert_result =
      locked_map.get().emplace(candidate_persistent_id, VertexId::Invalid());

  // If the persistent id already exists, then throw an exception
  if (insert_result.second == false)
    throw std::runtime_error(
        "Persistent ID already exists when trying to add vertex");

  // Otherwise, insert the vertex and return a pointer to it.
  VertexPtr vp = Graph::addVertex(runId);
  vp->setKeyFrameTime(time);
  vp->setPersistentId(stamp, robot);
  insert_result.first->second = vp->id();
#if 0
  // process the message stream queues
  runs_->at(runId)->processMsgQueue(vp);
#endif
  return vp;
}

void RCGraph::load(const RunFilter& r) {
  LockGuard lck(mtx_);

  CLOG(DEBUG, "pose_graph") << "Loading pose graph.";

  loadIndex();
  loadRuns(r);
}

void RCGraph::loadIndex() {
  LockGuard lck(mtx_);

  storage::DataStreamReader<vtr_messages::msg::GraphRunList> reader{
      fs::path{filePath_} / "graph_index"};
  auto msg_ptr = reader.readAtIndex(1);
  msg_ = msg_ptr->get<vtr_messages::msg::GraphRunList>();
  id_ = msg_.graph_id;

  std::stringstream ss;
  ss << "Loading pose graph index from message" << std::endl;
  ss << "- graph id (not used right now): " << msg_.graph_id << std::endl;
  ss << "- graph last run id (might not be set): " << msg_.last_run
     << std::endl;
  ss << "- graph run relative paths" << std::endl;
  for (const auto& rpath : msg_.run_rpath) ss << "  - " << rpath << std::endl;

  CLOG(DEBUG, "pose_graph") << ss.str();
}

void RCGraph::loadRuns(const RunFilter& r) {
  LockGuard lck(mtx_);

  CLOG(DEBUG, "pose_graph") << "Loading pose graph runs.";

  loadRunIndexes(r);
  for (auto it = runs_->begin(); it != runs_->end(); ++it)
    it->second->load(*vertices_, *edges_, r);
  buildSimpleGraphAndAddVertexNeighbors();
  buildPersistentMap();
}

void RCGraph::loadRunIndexes(const RunFilter& r) {
  LockGuard lck(mtx_);
  runs_->clear();

  std::stringstream ss;
  ss << "Loading pose graph runs from graph index msg" << std::endl
     << "Run paths:";

  for (unsigned i = 0; i < msg_.run_rpath.size(); ++i) {
    auto rpath = msg_.run_rpath[i];
    ss << " " << rpath << ";";

    if (!fs::exists(fs::path{filePath_} / rpath)) {
      std::stringstream err;
      err << "Run " << rpath << " is missing.";
      CLOG(ERROR, "pose_graph") << err.str();
      throw std::runtime_error(err.str());
    }

    RCRun::Ptr tmp_run(RCRun::MakeShared(fs::path{filePath_} / rpath));
    if (tmp_run->graphId() != id_) {
      std::stringstream err;
      err << "The graph index is broken; graph " << this->id_
          << " points to run file ";
      err << tmp_run->filePath() << " with graph id " << tmp_run->graphId();
      throw std::runtime_error(err.str());
    }

    if (r.size() == 0 || common::utils::contains(r, tmp_run->id())) {
      runs_->insert(std::make_pair(tmp_run->id(), tmp_run));
      currentRun_ = runs_->at(tmp_run->id());
    }
  }

  if (msg_.last_run == -1 && currentRun_ != nullptr)
    msg_.last_run = currentRun_->id();

  lastRunIdx_ = msg_.last_run;

  ss << std::endl << "Setting last run to " << lastRunIdx_;
  CLOG(DEBUG, "pose_graph") << ss.str();
}

#if 0
void RCGraph::saveWorkingIndex() {
  LockGuard lck(mtx_);

  robochunk::base::DataOutputStream ostream;

  // Recompute the file paths at save time to avoid zero-length runs
  msg_.clear_runrpath();
  auto N = robochunk::util::split_directory(filePath_).size();
  for (auto&& it : *runs_) {
    if (it.second->vertices().size() > 0) {
      msg_.add_runrpath(it.second->filePath().substr(N));
    }
  }

  ostream.openStream(filePath_ + ".working", true);
  ostream.serialize(msg_);
  ostream.closeStream();
}

void RCGraph::saveWorkingRuns() {
  LockGuard lck(mtx_);

  for (auto it = runs_->begin(); it != runs_->end(); ++it) {
    it->second->saveWorking();
  }
}

void RCGraph::saveWorking() {
  LockGuard lck(mtx_);

  saveWorkingIndex();
  saveWorkingRuns();
}
#endif

void RCGraph::save(bool force) {
  LockGuard lck(mtx_);
  CLOG(INFO, "pose_graph") << "Saving graph (force=" << force << ")...";

  CLOG(DEBUG, "pose_graph") << "Saving unwritten vertex data.";
  if (currentRun_ != nullptr && !currentRun_->readOnly()) {
    for (auto&& it : currentRun_->vertices()) {
      if (!it.second->isDataSaved()) it.second->write();
    }
  }

  saveIndex();
  saveRuns(force);

  CLOG(INFO, "pose_graph") << "Saving graph complete.";
}

void RCGraph::saveIndex() {
  LockGuard lck(mtx_);

  std::stringstream ss;
  ss << "Saving graph index to message" << std::endl;
  ss << "- graph id (not used right now): " << msg_.graph_id << std::endl;
  ss << "- graph last run id (might not be set): " << msg_.last_run
     << std::endl;

  // Recompute the file paths at save time to avoid zero-length runs
  msg_.run_rpath.clear();
  ss << "- graph run relative paths" << std::endl;
  for (auto&& it : *runs_) {
    if (it.second->vertices().size() > 0) {
      const auto run_rpath =
          fs::relative(fs::path{it.second->filePath()}, fs::path{filePath_});
      msg_.run_rpath.push_back(run_rpath);
      ss << "  - " << run_rpath.string() << std::endl;
    }
  }

  CLOG(DEBUG, "pose_graph") << ss.str();

  storage::DataStreamWriter<vtr_messages::msg::GraphRunList> writer{
      fs::path{filePath_} / "graph_index"};
  writer.write(msg_);
}

void RCGraph::saveRuns(bool force) {
  LockGuard lck(mtx_);

  CLOG(DEBUG, "pose_graph") << "Saving runs in the graph";

  for (auto it = runs_->begin(); it != runs_->end(); ++it) {
    // Don't save empty runs
    if (it->second->vertices().size() > 0) {
      it->second->save(force);
    } else {
      CLOG(WARNING, "pose_graph") << "Not saving run with id "
                                  << it->second->id() << "because it is empty.";
    }
  }
}

RCGraph::RunIdType RCGraph::addRun(IdType robotId, bool ephemeral, bool extend,
                                   bool dosave) {
  LockGuard lck(mtx_);

  removeEphemeralRuns();
  if (ephemeral) {
    // We don't increase the last run index, because we expect to erase this
    // run shortly
    currentRun_ = RunType::MakeShared(lastRunIdx_ + 1, id_);
    currentRun_->setRobotId(robotId);
    runs_->insert({lastRunIdx_ + 1, currentRun_});

    // We still raise the callbacks, because we need to register streams even
    // if they don't point to data...
    callback_->runAdded(currentRun_);

    CLOG(INFO, "pose_graph") << "Adding **EPHEMERAL** run " << lastRunIdx_ + 1;
  } else if (currentRun_ == nullptr ||
             (!extend && currentRun_->vertices().size() > 0)) {
    // Save before doing anything.  This ensures that only the current run
    // will have changes that need saving.
    if (dosave) save();

    // set the streams in the previous run to read only.
    if (currentRun_ != nullptr) currentRun_->setReadOnly();
    RunIdType newRunId = ++lastRunIdx_;
    CLOG(INFO, "pose_graph") << "[RCGraph] Adding run " << newRunId;
    msg_.last_run = lastRunIdx_;

    std::stringstream ss;
    ss << "run_" << std::setfill('0') << std::setw(6) << newRunId;
    ss << "/graph_" << std::setfill('0') << std::setw(2) << id_;
    currentRun_ = RunType::MakeShared(fs::path{filePath_} / fs::path{ss.str()},
                                      newRunId, id_);

    currentRun_->setRobotId(robotId);
    runs_->insert({newRunId, currentRun_});
    callback_->runAdded(currentRun_);
  } else if (extend) {
    CLOG(WARNING, "pose_graph")
        << "[RCGraph] Run already exists, extending the existing run";
  } else {
    CLOG(WARNING, "pose_graph")
        << "[RCGraph] Added a new run while the current run was "
           "empty; returning the existing run";
  }
  return currentRun_->id();
}

void RCGraph::removeEphemeralRuns() {
  if (runs_->size() > 0) {
    auto it = runs_->rbegin();
    if (it->second->isEphemeral()) {
      CLOG(INFO, "pose_graph") << "Deleting ephemeral run " << it->first;
      for (auto&& jt : it->second->edges(Temporal)) edges_->erase(jt.first);

      if (it->second->edges(Spatial).size() > 0) {
        CLOG(ERROR, "pose_graph")
            << "An ephemeral run had spatial edges... this cannot be "
               "completely cleaned up without restarting!!";
        throw;
        for (auto&& jt : it->second->edges(Spatial)) edges_->erase(jt.first);
      }

      runs_->erase(it->first);
      currentRun_ = runs_->rbegin()->second;
    }
  }
}

void RCGraph::buildSimpleGraphAndAddVertexNeighbors() {
  LockGuard lck(mtx_);

  if (graph_.numberOfEdges() > 0) {
    std::string err{"Cannot re-link edges on an existing graph!"};
    CLOG(ERROR, "pose_graph") << err;
    throw std::runtime_error{err};
  }

  // First add all vertices to the simple graph in case there are isolated
  // vertices.
  for (auto it = vertices_->begin(); it != vertices_->end(); ++it)
    graph_.addVertex(it->second->simpleId());

  // Add all edges to the simple graph as well as vertices they are connected
  // to.
  for (auto it = edges_->begin(); it != edges_->end(); ++it) {
    if (vertices_->find(it->second->from()) != vertices_->end() &&
        vertices_->find(it->second->to()) != vertices_->end()) {
      vertices_->at(it->second->from())->addEdge(it->second->id());
      vertices_->at(it->second->to())->addEdge(it->second->id());
      graph_.addEdge(it->second->from(), it->second->to());
    } else {
      CLOG(ERROR, "pose_graph")
          << " Could not link vertices " << it->second->from() << " and "
          << it->second->to();
      CLOG(ERROR, "pose_graph")
          << "Contains " << it->second->from() << " "
          << (vertices_->find(it->second->from()) != vertices_->end());
      CLOG(ERROR, "pose_graph")
          << "Contains " << it->second->to() << " "
          << (vertices_->find(it->second->to()) != vertices_->end());
      throw std::runtime_error{"Constructing simple graph failed."};
    }
  }
}

void RCGraph::buildPersistentMap() {
  // Lock and initialize the map
  auto locked_map = persistent_map_->locked();
  auto& map = locked_map.get();
  map.clear();
  map.reserve(numberOfVertices());

  // add all the vertices to the map
  for (auto it = vertices_->begin(); it != vertices_->end(); ++it) {
    auto& vertex = it->second;
    auto vertex_id = vertex->id();
    auto stamp = vertex->keyFrameTime().nanoseconds_since_epoch;
    auto robot = runs_->at(vertex->id().majorId())->robotId();
    vertex->setPersistentId(stamp, robot);
    map.emplace(vertex->persistentId(), vertex_id);
  }
}

#if 0
void RCGraph::reindexStream(const RunIdType& run_id,
                            const std::string& stream_name,
                            const WindowType& wType) {
  if (runs_ != nullptr && runs_->find(run_id) != runs_->end()) {
    runs_->at(run_id)->reindexStream(stream_name, wType);
  } else {
    CLOG(WARNING, "pose_graph") << "[RCGraph::reindexStream] Run " << run_id
                 << " was not in the run map.";
  }
}
#endif
void RCGraph::halt() {
  LockGuard lck(mtx_);
#if false
  // Properly purge any empty runs and their data
  for (auto it = runs_->rbegin(); it != runs_->rend(); ++it) {
    if (it->second->vertices().size() == 0) {
      std::string rdir = robochunk::util::split_directory(
          robochunk::util::split_directory(it->second->filePath()));
      CLOG(INFO, "pose_graph") << "Removing empty run: " << rdir;

      // If this was the last run, we can re-use the index later
      if (it->second->id() == lastRunIdx_) {
        --lastRunIdx_;
      }

      // Remove the actual directory
      std::string cmd("rm -rf " + rdir);
      int rc = std::system(cmd.c_str());

      if (rc != 0) {
        CLOG(ERROR, "pose_graph") << "Could not remove run directory: " << rdir;
      }
    }
  }
#endif
  save();
}
#if 0
template class Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;
typedef Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>> __GraphRC;

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, __GraphRC)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, __GraphRC)

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, RCGraph)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, RCGraph)
#endif
}  // namespace pose_graph
}  // namespace vtr
