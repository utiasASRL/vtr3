#if 0
#define RCGRAPH_NO_EXTERN
#endif

#include <filesystem>
#include <iomanip>  // \todo (yuchen) This is needed for setw/setfill, but should be included in other packages already.

#include <vtr_messages/msg/graph_persistent_id.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {

RCGraph::Ptr RCGraph::LoadOrCreate(const std::string& filePath,
                                   const IdType& id) {
  if (fs::exists(filePath)) {
    LOG(INFO) << "Graph exists, loading....";
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

  auto candidate_persistent_id = vtr_messages::msg::GraphPersistentId();
  candidate_persistent_id.stamp = stamp;
  candidate_persistent_id.robot = robot;

  auto locked_map = persistent_map_.locked();
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
}

void RCGraph::loadRuns(const RunFilter& r) {
  LockGuard lck(mtx_);

  loadRunIndexes(r);
  for (auto it = runs_->begin(); it != runs_->end(); ++it)
    it->second->load(*vertices_, *edges_, r);
  linkEdgesInternal();
  buildPersistentMap();
}

void RCGraph::loadRunIndexes(const RunFilter& r) {
  LockGuard lck(mtx_);
  runs_->clear();

  for (unsigned i = 0; i < msg_.run_rpath.size(); ++i) {
    auto rpath = msg_.run_rpath[i];
    if (!fs::exists(fs::path{filePath_} / rpath)) {
      LOG(INFO) << "[RCGraph] Skipped missing run: " << rpath;
      continue;
    }

    RCRun::Ptr tmp_run(RCRun::MakeShared(fs::path{filePath_} / rpath));
    if (tmp_run->graphId() != id_) {
      std::stringstream ss;
      ss << "The graph index is broken; graph " << this->id_
         << " points to run file ";
      ss << tmp_run->filePath() << " with graph id " << tmp_run->graphId();
      throw std::runtime_error(ss.str());
    }

    if (r.size() == 0 || common::utils::contains(r, tmp_run->id())) {
      runs_->insert(std::make_pair(tmp_run->id(), tmp_run));
      currentRun_ = runs_->at(tmp_run->id());
    }
  }

  if (msg_.last_run == -1 && currentRun_ != nullptr)
    msg_.last_run = currentRun_->id();

  lastRunIdx_ = msg_.last_run;
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
  LOG(INFO) << "Saving graph (force=" << force << ")...";

  LockGuard lck(mtx_);
  // save off unwritten vertex data
  if (currentRun_ != nullptr && !currentRun_->readOnly()) {
    for (auto&& it : currentRun_->vertices()) {
      if (!it.second->isDataSaved())
        it.second->write();
    }
  }

  saveIndex();
  saveRuns(force);

  LOG(INFO) << "Saving graph complete.";
}

void RCGraph::saveIndex() {
  LockGuard lck(mtx_);

  /// robochunk::base::DataOutputStream ostream;
  ///
  /// if (robochunk::util::file_exists(filePath_)) {
  ///   if (robochunk::util::file_exists(filePath_ + ".tmp")) {
  ///     std::remove((filePath_ + ".tmp").c_str());
  ///   }
  ///   robochunk::util::move_file(filePath_, filePath_ + ".tmp");
  /// }

  // Recompute the file paths at save time to avoid zero-length runs
  /// msg_.clear_runrpath();
  /// auto N = robochunk::util::split_directory(filePath_).size() + 1;
  /// for (auto&& it : *runs_) {
  ///   if (it.second->vertices().size() > 0) {
  ///     msg_.add_runrpath(it.second->filePath().substr(N));
  ///   }
  /// }
  msg_.run_rpath.clear();
  for (auto&& it : *runs_) {
    if (it.second->vertices().size() > 0)
      msg_.run_rpath.push_back(
          fs::relative(fs::path{it.second->filePath()}, fs::path{filePath_}));
  }

  /// ostream.openStream(filePath_, true);
  /// ostream.serialize(msg_);
  /// ostream.closeStream();
  ///
  /// if (robochunk::util::file_exists(filePath_ + ".tmp")) {
  ///   std::remove((filePath_ + ".tmp").c_str());
  /// }
  storage::DataStreamWriter<vtr_messages::msg::GraphRunList> writer{
      fs::path{filePath_} / "graph_index"};
  writer.write(msg_);
}

void RCGraph::saveRuns(bool force) {
  LockGuard lck(mtx_);

  for (auto it = runs_->begin(); it != runs_->end(); ++it) {
    // Don't save empty runs
    if (it->second->vertices().size() > 0) {
      it->second->save(force);
    } else {
      LOG(WARNING) << "Not saving run because it is empty";
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
    callbackManager_->runAdded(currentRun_);

    LOG(INFO) << "Adding **EPHEMERAL** run " << lastRunIdx_ + 1;
  } else if (currentRun_ == nullptr ||
             (!extend && currentRun_->vertices().size() > 0)) {
    // Save before doing anything.  This ensures that only the current run
    // will have changes that need saving.
    if (dosave)
      save();

    // set the streams in the previous run to read only.
    if (currentRun_ != nullptr)
      currentRun_->setReadOnly();
    RunIdType newRunId = ++lastRunIdx_;
    LOG(INFO) << "[RCGraph] Adding run " << newRunId;
    msg_.last_run = lastRunIdx_;

    std::stringstream ss;
    ss << "run_" << std::setfill('0') << std::setw(6) << newRunId;
    ss << "/graph_" << std::setfill('0') << std::setw(2) << id_;
    currentRun_ = RunType::MakeShared(fs::path{filePath_} / fs::path{ss.str()},
                                      newRunId, id_);

    currentRun_->setRobotId(robotId);
    runs_->insert({newRunId, currentRun_});
    callbackManager_->runAdded(currentRun_);
  } else if (extend) {
    LOG(WARNING) << "[RCGraph] Run already exists, extending the existing run";
  } else {
    LOG(WARNING) << "[RCGraph] Added a new run while the current run was "
                    "empty; returning the existing run";
  }
  return currentRun_->id();
}

void RCGraph::removeEphemeralRuns() {
  if (runs_->size() > 0) {
    auto it = runs_->rbegin();
    if (it->second->isEphemeral()) {
      LOG(INFO) << "Deleting ephemeral run " << it->first;
      for (auto&& jt : it->second->edges(Temporal)) edges_->erase(jt.first);

      if (it->second->edges(Spatial).size() > 0) {
        LOG(ERROR) << "An ephemeral run had spatial edges... this cannot be "
                      "completely cleaned up without restarting!!";
        for (auto&& jt : it->second->edges(Spatial)) edges_->erase(jt.first);
      }

      runs_->erase(it->first);
      currentRun_ = runs_->rbegin()->second;
    }
  }
}

vtr_messages::msg::GraphPersistentId RCGraph::toPersistent(
    const VertexIdType& vid) const {
  return at(vid)->persistentId();
}

auto RCGraph::fromPersistent(
    const vtr_messages::msg::GraphPersistentId& pid) const -> VertexIdType {
  try {
    return persistent_map_.locked().get().at(pid);
  } catch (...) {
    LOG(ERROR) << "Could not find persistent id: stamp: " << pid.stamp << ".\n";
#if false
    LOG(ERROR) << "Could not find " << pid.DebugString() << ".\n"
               << el::base::debug::StackTrace();
#endif
    throw;
  }
  return VertexIdType::Invalid();  // Should not get here
}

void RCGraph::linkEdgesInternal() {
  LockGuard lck(mtx_);

  if (graph_.numberOfEdges() > 0) {
    LOG(ERROR) << "[RCGraph::linkEdgesInternal] Cannot re-link edges on an "
                  "existing graph!";
    return;
  }

  for (auto it = edges_->begin(); it != edges_->end(); ++it) {
    if (vertices_->find(it->second->from()) != vertices_->end() &&
        vertices_->find(it->second->to()) != vertices_->end()) {
      vertices_->at(it->second->from())->addEdge(it->second->id());
      vertices_->at(it->second->to())->addEdge(it->second->id());
      graph_.addEdge(it->second->from(), it->second->to());
    } else {
      LOG(ERROR) << " Could not link vertices " << it->second->from() << " and "
                 << it->second->to();
      LOG(ERROR) << "Contains " << it->second->from() << " "
                 << (vertices_->find(it->second->from()) != vertices_->end());
      LOG(ERROR) << "Contains " << it->second->to() << " "
                 << (vertices_->find(it->second->to()) != vertices_->end());
    }
  }
}

void RCGraph::buildPersistentMap() {
  // Lock and initialize the map
  auto locked_map = persistent_map_.locked();
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
    LOG(WARNING) << "[RCGraph::reindexStream] Run " << run_id
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
      LOG(INFO) << "Removing empty run: " << rdir;

      // If this was the last run, we can re-use the index later
      if (it->second->id() == lastRunIdx_) {
        --lastRunIdx_;
      }

      // Remove the actual directory
      std::string cmd("rm -rf " + rdir);
      int rc = std::system(cmd.c_str());

      if (rc != 0) {
        LOG(ERROR) << "Could not remove run directory: " << rdir;
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
