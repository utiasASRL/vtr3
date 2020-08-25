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

RCGraph::Ptr RCGraph::MakeShared() { return Ptr(new RCGraph()); }
#if 0
RCGraph::Ptr RCGraph::MakeShared(const std::string& filePath,
                                 const IdType& id) {
  return Ptr(new RCGraph(filePath, id));
}

RCGraph::Ptr RCGraph::MakeShared(const std::string& filePath) {
  return Ptr(new RCGraph(filePath));
}

RCGraph::Ptr RCGraph::LoadOrCreate(const std::string& filePath,
                                   const IdType& id) {
  if (robochunk::util::file_exists(filePath)) {
    LOG(INFO) << "It exists, loading....";
    auto graph = Ptr(new RCGraph(filePath));
    graph->load();
    return graph;
  } else {
    return Ptr(new RCGraph(filePath, id));
  }
}
#endif

RCGraph::RCGraph()
    : Base(),
      RCGraphBase(),
      GraphType(),
      filePath_(""),
      /// msg_(asrl::graph_msgs::RunList())
      msg_() {
  msg_.last_run = uint32_t(-1);
}

#if 0
RCGraph::RCGraph(RCGraph&& other)
    : GraphType(std::move(other)),
      filePath_(std::move(other.filePath_)),
      msg_(other.msg_) {}
RCGraph& RCGraph::operator=(RCGraph&& other) {
  GraphType::operator=(std::move(other));
  this->filePath_ = std::move(other.filePath_);
  this->msg_ = std::move(other.msg_);
  return *this;
}
#endif
RCGraph::RCGraph(const std::string& filePath, const IdType& id)
    : Base(id),
      RCGraphBase(id),
      GraphType(id),
      filePath_(filePath),
      /// msg_(asrl::graph_msgs::RunList())
      msg_() {
  msg_.graph_id = this->id_;
  msg_.last_run = uint32_t(-1);
#if 0
  robochunk::util::create_directories(
      robochunk::util::split_directory(filePath_));
  saveIndex();
#endif
}
#if 0
RCGraph::RCGraph(const std::string& filePath)
    : Base(),
      RCGraphBase(),
      GraphType(),
      filePath_(filePath),
      msg_(asrl::graph_msgs::RunList()) {
  msg_.set_graphid(this->id_);
  msg_.set_lastrun(uint32_t(-1));
  robochunk::util::create_directories(
      robochunk::util::split_directory(filePath_));
  // loadIndex();
}
#endif
/// auto RCGraph::addVertex(const robochunk::std_msgs::TimeStamp& time)
///     -> VertexPtr {
///   return addVertex(time, currentRun_->id());
/// }
auto RCGraph::addVertex(const vtr_messages::msg::TimeStamp& time) -> VertexPtr {
  return addVertex(time, currentRun_->id());
}

/// auto RCGraph::addVertex(const robochunk::std_msgs::TimeStamp& time,
///                         const RunIdType& runId) -> VertexPtr {
///   LockGuard lock(mtx_);
///
///   // Check that the persistent id doesn't already exist before adding the
///   // vertex.
///   uint64_t stamp = time.nanoseconds_since_epoch();
///   uint32_t robot = runs_->at(runId)->robotId();
///
///   graph_msgs::PersistentId candidate_persistent_id;
///   candidate_persistent_id.set_stamp(stamp);
///   candidate_persistent_id.set_robot(robot);
///
///   auto locked_map = persistent_map_.locked();
///   auto insert_result =
///       locked_map.get().emplace(candidate_persistent_id,
///       VertexId::Invalid());
///
///   // If the persistent id already exists, then throw an exception
///   if (insert_result.second == false) {
///     throw std::runtime_error(
///         "Persistent ID already exists when trying to add vertex");
///   }
///
///   // Otherwise, insert the vertex and return a pointer to it.
///   VertexPtr vp = Graph::addVertex(runId);
///   vp->setKeyFrameTime(time);
///   vp->setPersistentId(stamp, robot);
///   insert_result.first->second = vp->id();
///
///   // process the message stream queues
///   runs_->at(runId)->processMsgQueue(vp);
///
///   return vp;
/// }
auto RCGraph::addVertex(const vtr_messages::msg::TimeStamp& time,
                        const RunIdType& runId) -> VertexPtr {
  LockGuard lock(mtx_);

  // Check that the persistent id doesn't already exist before adding the
  // vertex.
  uint64_t stamp = time.nanoseconds_since_epoch;
  uint32_t robot = runs_->at(runId)->robotId();

  /// graph_msgs::PersistentId candidate_persistent_id;
  /// candidate_persistent_id.set_stamp(stamp);
  /// candidate_persistent_id.set_robot(robot);
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
#if 0
void RCGraph::loadIndex() {
  LockGuard lck(mtx_);
  robochunk::base::DataInputStream istream;
  msg_.Clear();

  istream.openStream(filePath_);
  istream.deserialize(msg_);
  istream.closeStream();

  this->id_ = msg_.graphid();
}
/// @brief Load the indexes of each run to inspect descriptions

void RCGraph::loadRunIndexes(const RunFilter& r) {
  LockGuard lck(mtx_);
  runs_->clear();

  for (int i = 0; i < msg_.runrpath_size(); ++i) {
    std::string rpath =
        robochunk::util::split_directory(filePath_) + "/" + msg_.runrpath(i);
    if (!robochunk::util::file_exists(rpath)) {
      LOG(INFO) << "[RCGraph] Skipped missing run: " << rpath;
      continue;
    }

    RCRun::Ptr tmpRun(RCRun::MakeShared(rpath));
    if (tmpRun->graphId() != id_) {
      std::stringstream ss;
      ss << "The graph index is broken; graph " << this->id_
         << " points to run file ";
      ss << tmpRun->filePath() << " with graph id " << tmpRun->graphId();
      throw std::runtime_error(ss.str());
    }

    if (r.size() == 0 || asrl::common::utils::contains(r, tmpRun->id())) {
      (void)runs_->insert(std::make_pair(tmpRun->id(), tmpRun));
      currentRun_ = runs_->at(tmpRun->id());
    }
  }

  if (!msg_.has_lastrun()) {
    if (currentRun_ != nullptr) {
      msg_.set_lastrun(currentRun_->id());
    } else {
      msg_.set_lastrun(-1);
    }
  }

  lastRunIdx_ = msg_.lastrun();
}
/// @brief Deep load runs and their vertex/edge data

void RCGraph::loadRuns(const RunFilter& r) {
  LockGuard lck(mtx_);

  loadRunIndexes(r);
  for (auto it = runs_->begin(); it != runs_->end(); ++it) {
    it->second->load(*vertices_, *edges_, r);
  }
  linkEdgesInternal();
  buildPersistentMap();
}
/// @brief Deep load all levels of index data

void RCGraph::load(const RunFilter& r) {
  LockGuard lck(mtx_);

  loadIndex();
  loadRuns(r);
}

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

void RCGraph::saveIndex() {
  LockGuard lck(mtx_);

  robochunk::base::DataOutputStream ostream;

  if (robochunk::util::file_exists(filePath_)) {
    if (robochunk::util::file_exists(filePath_ + ".tmp")) {
      std::remove((filePath_ + ".tmp").c_str());
    }
    robochunk::util::move_file(filePath_, filePath_ + ".tmp");
  }

  // Recompute the file paths at save time to avoid zero-length runs
  msg_.clear_runrpath();
  auto N = robochunk::util::split_directory(filePath_).size() + 1;
  for (auto&& it : *runs_) {
    if (it.second->vertices().size() > 0) {
      msg_.add_runrpath(it.second->filePath().substr(N));
    }
  }

  ostream.openStream(filePath_, true);
  ostream.serialize(msg_);
  ostream.closeStream();

  if (robochunk::util::file_exists(filePath_ + ".tmp")) {
    std::remove((filePath_ + ".tmp").c_str());
  }
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
#endif
void RCGraph::save(bool force) {
  LOG(INFO) << "Saving graph..."
            << "(force=" << force << ")";
  LockGuard lck(mtx_);
#if 0
  // save off unwritten vertex data
  if (currentRun_ != nullptr && !currentRun_->readOnly()) {
    for (auto&& it : currentRun_->vertices()) {
      if (!it.second->isDataSaved()) {
        it.second->write();
      }
    }
  }

  saveIndex();
  saveRuns(force);
#endif
  LOG(INFO) << "Saving graph complete.";
}

std::string RCGraph::filePath() const { return filePath_; }

RCGraph::RunIdType RCGraph::addRun(IdType robotId, bool ephemeral, bool extend,
                                   bool dosave) {
  LockGuard lck(mtx_);

  removeEphemeralRuns();
  if (ephemeral) {
    // We don't increase the last run index, because we expect to erase this
    // run shortly
    currentRun_ = RunType::MakeShared(lastRunIdx_ + 1, this->id_);
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
    if (dosave) save();

    // set the streams in the previous run to read only.
    if (currentRun_ != nullptr) currentRun_->setReadOnly();
    RunIdType newRunId = ++lastRunIdx_;
    LOG(INFO) << "[RCGraph] Adding run " << newRunId;
    msg_.last_run = lastRunIdx_;

    /// std::stringstream ss;
    /// ss << "/run_" << std::setfill('0') << std::setw(6) << newRunId;
    /// ss << "/graph_" << std::setfill('0') << std::setw(2) << this->id_;
    /// ss << "/run.proto";
    /// //    msg_.add_runrpath(ss.str());
    /// currentRun_ = RunType::MakeShared(
    ///     robochunk::util::split_directory(filePath_) + ss.str(),
    ///     newRunId, this->id_);
    std::stringstream ss;
    ss << "run_" << std::setfill('0') << std::setw(6) << newRunId;
    ss << "/graph_" << std::setfill('0') << std::setw(2) << this->id_;
    ss << "/run.proto";
    currentRun_ = RunType::MakeShared(fs::path{filePath_} / fs::path{ss.str()},
                                      newRunId, this->id_);

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

#if 0
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
    uint64_t stamp = vertex->keyFrameTime().nanoseconds_since_epoch();
    uint32_t robot = runs_->at(vertex->id().majorId())->robotId();
    vertex->setPersistentId(stamp, robot);
    map.emplace(vertex->persistentId(), vertex_id);
  }
}

void RCGraph::registerVertexStream(const RCGraph::RunIdType& run_id,
                                   const std::string& stream_name,
                                   bool points_to_data,
                                   const RegisterMode& mode) {
  if (runs_ != nullptr && runs_->find(run_id) != runs_->end()) {
    auto& run = runs_->at(run_id);
    if (!run->hasVertexStream(stream_name)) {
      run->registerVertexStream(stream_name, points_to_data, mode);
    }
  } else {
    LOG(WARNING) << "[RCGraph::registerVertexStream] Run " << run_id
                 << " was not in the run map.";
  }
}

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

const asrl::graph_msgs::MapInfo& RCGraph::mapInfo() const { return msg_.map(); }

asrl::graph_msgs::MapInfo* RCGraph::mutableMapInfo() {
  return msg_.mutable_map();
}

void RCGraph::setMapInfo(const asrl::graph_msgs::MapInfo& map) {
  msg_.mutable_map()->CopyFrom(map);
}
/// @brief Removes any empty runs and associated folders from the graph.

void RCGraph::halt() {
  LockGuard lck(mtx_);

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

  save();
}

template class Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;
typedef Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>> __GraphRC;

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, __GraphRC)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, __GraphRC)

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, RCGraph)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, RCGraph)
#endif
}  // namespace pose_graph
}  // namespace vtr
