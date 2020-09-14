#include <filesystem>

#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>

/// #include <robochunk/base/BaseChunkSerializer.hpp>
/// #include <robochunk/base/SerializerFactory.hpp>
#include <vtr_pose_graph/robochunk/base/chunk_serializer.hpp>
#include <vtr_pose_graph/robochunk/base/data_stream.hpp>

#if 0
#include <sstream>
#include <string>

#include <robochunk/base/NonUniformIndexDeserializer.h>
#include <robochunk/base/NonUniformIndexSerializerDecorator.h>
#include <robochunk/base/TimeSerializerDecorator.h>
#include <robochunk/base/UniformIndexSerializerDecorator.h>

#include <stdlib.h>

//#include <asrl/pose_graph/index/RCGraph.hpp>
#endif

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {
#if 0
void ensureFolder(const std::string& path) {
  if (!robochunk::util::file_exists(robochunk::util::split_directory(path))) {
    robochunk::util::create_directories(robochunk::util::split_directory(path));
  }
}
#endif
RCRun::Ptr RCRun::MakeShared() { return Ptr(new RCRun()); }
RCRun::Ptr RCRun::MakeShared(const IdType& runId, const IdType& graphId) {
  return Ptr(new RCRun(runId, graphId));
}
RCRun::Ptr RCRun::MakeShared(const std::string& filePath, const IdType& runId,
                             const IdType& graphId) {
  return Ptr(new RCRun(filePath, runId, graphId));
}
RCRun::Ptr RCRun::MakeShared(const std::string& filePath) {
  return Ptr(new RCRun(filePath));
}

RCRun::RCRun()
    : RunBase<RCVertex, RCEdge>(),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      robochunkStreams_(LockableStreamMapPtr(new LockableStreamMap())),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(""),
      /// msg_(asrl::graph_msgs::Run())
      msg_(),
      readOnly_(false)
#if 0
      edgeStreamNames_(LockableFieldMapPtrArray()),
      wasLoaded_(false),
#endif
{
#if 0
  for (auto it = edgeStreamNames_.begin(); it != edgeStreamNames_.end(); ++it) {
    *it = LockableFieldMapPtr(new LockableFieldMap());
  }
#endif
}

RCRun::RCRun(const IdType& runId, const IdType& graphId)
    : RunBase<RCVertex, RCEdge>(runId, graphId),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      robochunkStreams_(LockableStreamMapPtr(new LockableStreamMap())),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(""),
      /// msg_(asrl::graph_msgs::Run()),
      msg_(),
      readOnly_(true)
#if 0
      edgeStreamNames_(LockableFieldMapPtrArray()),
      wasLoaded_(false),
#endif
{
  msg_.id = runId;
  msg_.graph_id = graphId;
}

RCRun::RCRun(const std::string& filePath, const IdType& runId,
             const IdType& graphId)
    : RunBase<RCVertex, RCEdge>(runId, graphId),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      robochunkStreams_(LockableStreamMapPtr(new LockableStreamMap())),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(filePath),
      /// msg_(asrl::graph_msgs::Run()),
      msg_(),
      readOnly_(false)
#if 0
      edgeStreamNames_(LockableFieldMapPtrArray()),
      wasLoaded_(false),
#endif
{
#if 0
  for (auto it = edgeStreamNames_.begin(); it != edgeStreamNames_.end(); ++it)
    *it = LockableFieldMapPtr(new LockableFieldMap());
#endif
  msg_.graph_id = graphId_;
  msg_.id = id_;
  msg_.vertex_rpath = "vertex.proto";
  msg_.edge_rpaths.push_back("temporal_edge.proto");
  msg_.edge_rpaths.push_back("spatial_edge.proto");
#if 0
  robochunk::util::create_directories(
      robochunk::util::split_directory(filePath_));
  //  saveIndex();
#endif
}

RCRun::RCRun(const std::string& filePath)
    : RunBase<RCVertex, RCEdge>(),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      robochunkStreams_(LockableStreamMapPtr(new LockableStreamMap())),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(filePath),
      /// msg_(asrl::graph_msgs::Run()),
      msg_(),
      readOnly_(true),
#if 0
      edgeStreamNames_(LockableFieldMapPtrArray()),
      wasLoaded_(false),
#endif
      robotId_(-1) {
#if 0
  loadIndex();
#endif
}

#if 0
bool RCRun::loadIndex() {
  if (isEphemeral()) {
    return false;
  }

  robochunk::base::DataInputStream istream;
  msg_.Clear();

  istream.openStream(filePath_);
  istream.deserialize(msg_);
  istream.closeStream();

  id_ = msg_.id();
  graphId_ = msg_.graphid();
  robotId_ = msg_.robot_id();
  readOnly_ = true;

  // Load the manual flag
  if (msg_.has_containsmanual()) {
    manual_ = msg_.containsmanual();
    return false;
  } else {
    // We cannot compute it here as we might not have loaded edges yet...
    manual_ = false;
    return true;
  }
}

void RCRun::loadEdges(EdgePtrMapExtern& edgeDataMap,
                      const std::unordered_set<IdType>& runFilter) {
  if (isEphemeral()) {
    return;
  }

  for (int i = 0; i < msg_.edgerpaths_size(); ++i) {
    size_t idx = loadHeaderInternal(
        robochunk::util::split_directory(filePath_) + "/" + msg_.edgerpaths(i));

    size_t num = loadDataInternal(
        edgeDataMap, edges_[idx], edgeStreamNames_[idx],
        robochunk::util::split_directory(filePath_) + "/" + msg_.edgerpaths(i),
        runFilter);

    if (idx >= EdgeIdType::NumTypes() && num > 0) {
      throw std::invalid_argument(
          "These edge objects don't recognize the edge message types you are "
          "trying to load.");
    }
  }

  readOnly_ = true;
}

void RCRun::loadVertices(VertexPtrMapExtern& vertexDataMap) {
  if (isEphemeral()) {
    return;
  }

  loadDataInternal(
      vertexDataMap, vertices_, vertexStreamNames_,
      robochunk::util::split_directory(filePath_) + "/" + msg_.vertexrpath());
  readOnly_ = true;
}

void RCRun::saveWorkingIndex() {
  if (isEphemeral()) {
    return;
  }

  robochunk::base::DataOutputStream ostream;

  ostream.openStream(
      robochunk::util::split_directory(filePath_) + "/working/index.proto",
      true);
  ostream.serialize(msg_);
  ostream.closeStream();
}

void RCRun::saveWorkingEdges() {
  if (isEphemeral()) {
    return;
  }

  for (unsigned int i = 0; i < edges_.size(); ++i) {
    saveWorkingInternal(edges_[i], edgeStreamNames_[i],
                        robochunk::util::split_directory(filePath_));
  }
}

void RCRun::saveWorkingVertices() {
  if (isEphemeral()) {
    return;
  }

  saveWorkingInternal(vertices_, vertexStreamNames_,
                      robochunk::util::split_directory(filePath_));
}
#endif
void RCRun::saveIndex(bool force) {
  if (isEphemeral() || (readOnly_ && !force)) return;

  msg_.contains_manual = manual_;
#if 0
  robochunk::base::DataOutputStream ostream;

  if (robochunk::util::file_exists(filePath_)) {
    if (robochunk::util::file_exists(filePath_ + ".tmp")) {
      std::remove((filePath_ + ".tmp").c_str());
    }
    robochunk::util::move_file(filePath_, filePath_ + ".tmp");
  }

  ostream.openStream(filePath_, true);
  ostream.serialize(msg_);
  ostream.closeStream();

  if (robochunk::util::file_exists(filePath_ + ".tmp")) {
    std::remove((filePath_ + ".tmp").c_str());
  }
#endif
  readOnly_ = true;
}

void RCRun::saveVertices(bool force) {
  if (isEphemeral() || (readOnly_ && !force)) return;
#if 0
  saveDataInternal(
      vertices_, vertexStreamNames_,
      robochunk::util::split_directory(filePath_) + "/" + msg_.vertexrpath());
#endif
  readOnly_ = true;
}

void RCRun::saveEdges(bool force) {
  if (isEphemeral() || (readOnly_ && !force)) return;
#if 0
  for (unsigned int i = 0; i < edges_.size(); ++i) {
    saveDataInternal(
        edges_[i], edgeStreamNames_[i],
        robochunk::util::split_directory(filePath_) + "/" + msg_.edgerpaths(i));
  }
#endif
  readOnly_ = true;
}

#if 0
void RCRun::load(VertexPtrMapExtern& vertexDataMap,
                 EdgePtrMapExtern& edgeDataMap,
                 const std::unordered_set<IdType>& runFilter) {
  if (isEphemeral()) {
    return;
  }

  bool calcManual = loadIndex();
  loadVertices(vertexDataMap);
  loadEdges(edgeDataMap, runFilter);

  if (msg_.reindex()) {
    for (auto&& it :
         common::utils::getRefs(vertexStreamNames_->locked().get())) {
      reindexStream(it.get().first, WindowType::Before, false);
    }

    msg_.set_reindex(false);
  }
  if (calcManual) {
    this->computeManual();
  }
}

void RCRun::saveWorking() {
  if (isEphemeral()) {
    return;
  }

  robochunk::util::create_directories(
      robochunk::util::split_directory(filePath_) + "/working");

  saveWorkingIndex();
  saveWorkingVertices();
  saveWorkingEdges();
}
#endif
void RCRun::save(bool force) {
  if (isEphemeral() || (readOnly_ && !force)) {
    return;
  }

  saveIndex(true);
  saveVertices(true);
  saveEdges(true);

  //  boost::filesystem::remove_all(robochunk::util::split_directory(filePath_)
  //  + "/working");

  /// This is technically really bad, as someone could set the filePath_ of the
  /// run to something like:
  ///
  ///      ~/; sudo something_horrible; mkdir ./
  ///
  /// And it wouldn't even fail to run... But also this is internal code
  //  std::string cmd("rm -rf " + robochunk::util::split_directory(filePath_) +
  //  "/working"); int rc = std::system(cmd.c_str());
  //
  //  if (rc != 0) {
  //    throw std::runtime_error("Could not remove working directory: " +
  //    robochunk::util::split_directory(filePath_) + "/working");
  //  }
}

std::string RCRun::filePath() const { return filePath_; }
#if 0
void RCRun::setFilePath(const std::string& fpath) {
  if (wasLoaded_) {
    throw std::runtime_error(
        "You cannot change the index file after loading data!");
  }

  filePath_ = fpath;
}

bool RCRun::wasLoaded() const { return wasLoaded_; }

void RCRun::loadVertexStream(const std::string& streamName, uint64_t,
                             uint64_t) {
  // TODO: implement loading/unloading with a start/end index/time
  for (auto&& it : vertices_) {
    it.second->load(streamName);
  }
}

void RCRun::unloadVertexStream(const std::string& streamName, uint64_t,
                               uint64_t) {
  BaseIdType idx;
  {
    auto locked_vertex_stream_names = vertexStreamNames_->locked();
    auto stream_itr = locked_vertex_stream_names.get().find(streamName);
    if (stream_itr == locked_vertex_stream_names.get().end()) {
      LOG(WARNING) << "Stream " << streamName << " doesn't exist in run "
                   << id_;
      return;
    }
    idx = stream_itr->second;
  }

  // TODO: implement loading/unloading with a start/end index/time
  for (auto&& it : vertices_) {
    it.second->unload(streamName);
  }

  RobochunkIO& stream = robochunkStreams_->locked().get().at(idx);
  if (stream.first != nullptr) {
    auto guard = stream.lock(true, false);
    stream.first->close();
  }
}

size_t RCRun::loadHeaderInternal(const std::string& fpath) {
  typename EdgeType::HeaderMsg head;
  robochunk::base::DataInputStream istream;

  istream.openStream(fpath);
  istream.deserialize(head);
  istream.closeStream();

  return size_t(head.type());
}
#endif
void RCRun::setReadOnly() {
  if (isEphemeral()) return;

#if 0
  auto data_directory = robochunk::util::split_directory(
      robochunk::util::split_directory(filePath_));

  // Get stream names and indices so we can unlock the map.
  std::vector<FieldMap::value_type> streams;
  {
    auto locked_vertex_stream_names = vertexStreamNames_->locked();
    streams.reserve(locked_vertex_stream_names.get().size());
    for (auto name_itr = locked_vertex_stream_names.get().begin();
         name_itr != locked_vertex_stream_names.get().end(); ++name_itr) {
      streams.push_back(*name_itr);
    }
  }

  for (auto& stream : streams) {
    // reset the stream in read only
    auto& stream_index = stream.second;
    auto& stream_name = stream.first;

    // Lock the stream for the duration of resetting to read only
    {
      auto locked_robochunk_streams = robochunkStreams_->locked();
      RobochunkIO& roboio = locked_robochunk_streams.get().at(stream_index);
      auto guard = roboio.lock();

      // reset the serializer to null
      locked_robochunk_streams.get().at(stream_index).second.reset();
      // reset the deserializer
      //    LOG(DEBUG) << "Starting deserializer!!" << stream_name;
      locked_robochunk_streams.get()
          .at(stream_index)
          .first.reset(new robochunk::base::ChunkStream(data_directory,
                                                        "/" + stream_name));
    }

    // iterate through all of the vertices in this run and restart the stream.
    for (auto v_itr = vertices_.begin(); v_itr != vertices_.end(); ++v_itr) {
      //      LOG(INFO) << "Resetting bubble for : " << v_itr->first;
      v_itr->second->resetBubble(stream_name);
    }
  }
#endif
}
#if 0
void RCRun::setReadOnly(const std::string& stream) {
  if (isEphemeral()) {
    return;
  }

  // Check to make sure the stream is registered
  FieldMap::mapped_type idx;
  {
    auto locked_vertex_stream_names = vertexStreamNames_->locked();
    auto itr = locked_vertex_stream_names.get().find(stream);
    if (itr == locked_vertex_stream_names.get().end()) {
      LOG(WARNING) << "Stream " << stream
                   << " was not registered; not converted to read only.";
      return;
    }
    idx = itr->second;
  }

  {
    // Lock the stream for the duration of resetting to read only
    RobochunkIO& rc_stream = robochunkStreams_->locked().get().at(idx);
    auto guard = rc_stream.lock();

    // Resetting the serializer to a nullptr flushes everything to disk
    rc_stream.second.reset();

    // Create a new ChunkStream to read the data we just wrote out
    auto data_directory = robochunk::util::split_directory(
        robochunk::util::split_directory(filePath_));
    rc_stream.first.reset(
        new robochunk::base::ChunkStream(data_directory, "/" + stream));
  }

  // Reset all of the data bubbles to use the new ChunkStream
  for (auto v_itr = vertices_.begin(); v_itr != vertices_.end(); ++v_itr) {
    v_itr->second->resetBubble(stream);
  }
}

void RCRun::finalizeStream(const std::string& stream) {
  if (isEphemeral()) {
    return;
  }

  // Check to make sure the stream is registered
  auto itr = vertexStreamNames_->locked().get().find(stream);
  if (itr == vertexStreamNames_->locked().get().end()) {
    LOG(WARNING) << "Stream " << stream
                 << " was not registered; not flushed to disk.";
    return;
  }

  // Resetting the serializer to a nullptr flushes everything to disk and
  // prevents further writes
  RobochunkIO& roboio = robochunkStreams_->locked().get().at(itr->second);
  auto guard = roboio.lock(false, true);
  roboio.second.reset();
}
#endif

#if 0
void RCRun::reindexStream(const std::string& stream_name,
                          const WindowType& wType, bool overwrite) {
  if (isEphemeral()) {
    return;
  }

  FieldMap::mapped_type stream_idx;
  {
    auto locked_vertex_stream_names = vertexStreamNames_->locked();
    auto stream_itr = locked_vertex_stream_names.get().find(stream_name);
    if (stream_itr == locked_vertex_stream_names.get().end()) {
      LOG(WARNING) << "[RCRun::reindexStream] Stream " << stream_name
                   << " not reindexed because it is not registered.";
      return;
    }
    stream_idx = stream_itr->second;
  }

  using robochunk::msgs::RobochunkMessage;
  using robochunk::std_msgs::TimeStamp;

  auto data_directory = robochunk::util::split_directory(
      robochunk::util::split_directory(filePath_));

  RobochunkIO& rc_stream = robochunkStreams_->locked().get().at(stream_idx);
  auto guard = rc_stream.lock(true, false);
  auto stream = std::make_shared<robochunk::base::ChunkStream>(
      data_directory, "/" + stream_name);
  rc_stream.first = stream;

  stream->seek(0u, 0u);
  auto vertex = vertices_.begin(), vertex_end = vertices_.end();
  RobochunkMessage currentMsg;

  uint64_t cutoff = vertex->second->keyFrameTime().nanoseconds_since_epoch();
  int32_t start_idx = 0, end_idx = 0;

  // Seek through any data that happens before the first vertex; this is handled
  // as a special case
  stream->seek(vertex->second->keyFrameTime());
  stream->next(currentMsg);
  if (currentMsg.header().sensor_time_stamp().nanoseconds_since_epoch() <
      cutoff) {
    stream->next(currentMsg);
  }
  end_idx = currentMsg.header().sequence_id();

  switch (wType) {
    // Everything before the first vertex belongs to the first vertex
    case WindowType::Before: {
      if (end_idx > start_idx) {
        vertex->second->addStreamIndices(stream_name, {start_idx, end_idx - 1},
                                         overwrite);
      }
      // NOTE: vertex iterator is incremented; it is "ahead" of the data stream
      ++vertex;
      cutoff = vertex->second->keyFrameTime().nanoseconds_since_epoch();
      start_idx = end_idx;
      break;
    }
    // Everything before the first vertex is ignored
    case WindowType::After: {
      cutoff =
          std::next(vertex)->second->keyFrameTime().nanoseconds_since_epoch() -
          1;
      start_idx = end_idx;
      break;
    }
    // Everything before the first vertex is lumped in with the first vertex,
    // creating a "lopsided", centered window
    case WindowType::Center: {
      cutoff = (cutoff + std::next(vertex)
                             ->second->keyFrameTime()
                             .nanoseconds_since_epoch()) /
               2;
      break;
    }
  }

  // Loop through all messages and assign correct index values to all vertices
  while (stream->next(currentMsg) && vertex != vertex_end) {
    ++end_idx;
    uint64_t currentTime =
        currentMsg.header().sensor_time_stamp().nanoseconds_since_epoch();

    if (currentTime > cutoff) {
      // Only set index values if there was data in that time period
      if (end_idx > start_idx) {
        vertex->second->addStreamIndices(stream_name, {start_idx, end_idx - 1},
                                         overwrite);
      }

      start_idx = end_idx;
      ++vertex;

      // Update the time threshold based on window type
      if (vertex != vertex_end) {
        switch (wType) {
          case WindowType::Before: {
            // Use the current vertex, since the iterator is "ahead" of the data
            cutoff = vertex->second->keyFrameTime().nanoseconds_since_epoch();
            break;
          }
          case WindowType::After: {
            auto tmp = std::next(vertex);
            if (tmp != vertex_end) {
              // Use the next vertex, as the iterator is "in sync" with the data
              cutoff = tmp->second->keyFrameTime().nanoseconds_since_epoch();
            } else {
              // If there is no next vertex, exit the loop and handle trailing
              // data as a special case
              ++vertex;
            }
            break;
          }
          case WindowType::Center: {
            auto tmp = std::next(vertex);
            if (tmp != vertex_end) {
              // Use the current and next vertex; the iterator is "in sync" with
              // the data, but the cutoff is the midpoint between this vertex
              // and the next one
              cutoff =
                  (vertex->second->keyFrameTime().nanoseconds_since_epoch() +
                   tmp->second->keyFrameTime().nanoseconds_since_epoch()) /
                  2;
            } else {
              // If there is no next vertex, exit the loop and handle trailing
              // data as a special case
              ++vertex;
            }
            break;
          }
        }  // switch(wType)
      }    // vertex != vertex_end
    }      // currentTime > cutoff
  }        // while

  // For windows of type After/Center, all trailing data is lumped into the last
  // vertex
  if ((wType == WindowType::After || wType == WindowType::Center) &&
      vertex == vertex_end) {
    --vertex;
    while (stream->next(currentMsg)) {
      ++end_idx;
    }

    if (end_idx > start_idx) {
      vertex->second->addStreamIndices(stream_name, {start_idx, end_idx - 1},
                                       overwrite);
    }
  }
}
#endif
const std::shared_ptr<RCVertex>& RCRun::addVertex(const VertexIdType& v) {
  auto vertex = RunBase<RCVertex, RCEdge>::addVertex(v);
  // Pass the stream-map.
  vertex->setStreamNameMap(vertexStreamNames_);
#if 1
  vertex->setStreamMap(robochunkStreams_);
#endif
  vertex->setDataStreamMap(rosbag_streams_);
  // We have to re-look up because we need to return a direct reference :/
  return vertices_.at(vertex->id());
}
#if 0
void RCRun::processMsgQueue(RCVertex::Ptr vertex) {
  // Make sure the previous vertex exists
  if (vertices_.find(--vertex->id()) == vertices_.end()) {
    LOG(WARNING) << "processMsgQueue failed because it tried to insert at a "
                    "vertex that does not exist.";
    return;
  }

  // Define a lambda for getting the time-stamp from a robochunk message
  auto getRCStamp = [](robochunk::msgs::RobochunkMessage rc_msg) {
    return rc_msg.header().sensor_time_stamp().nanoseconds_since_epoch();
  };

  // Iterate over all stream buffers and add messages if they fall into the
  // right time range
  for (auto iter = streamBuffers_.begin(); iter != streamBuffers_.end();
       ++iter) {
    std::queue<RCMsg>& stream_queue = iter->second;
    std::queue<RCMsg> future_msg_queue;

    // Check to see if any messages need to be inserted
    while (!stream_queue.empty()) {
      std::lock_guard<std::mutex> lock(streamBufferLock_);

      // Message is inserted at or after the most recent vertex. Keep for later
      // processing.
      if (getRCStamp(stream_queue.front()) >=
          vertex->keyFrameTime().nanoseconds_since_epoch()) {
        future_msg_queue.push(stream_queue.front());
        stream_queue.pop();
      }
      // Message is older than the second most recent vertex. Try to find the
      // vertex to insert.
      else {
        // Iterate through the vertex list to see if we can find the correct
        // vertex to insert the experience
        VertexId tmp_vid = vertex->id();
        while (tmp_vid.minorId() > 0) {
          --tmp_vid;
          if (getRCStamp(stream_queue.front()) >=
              vertices_.at(tmp_vid)->keyFrameTime().nanoseconds_since_epoch()) {
            vertices_.at(tmp_vid)->insert(iter->first, stream_queue.front());
            break;
          }
        }
        stream_queue.pop();
      }
    }

    // copy future messages back into the queue
    std::lock_guard<std::mutex> lock(streamBufferLock_);
    stream_queue.swap(future_msg_queue);
  }

  return;
}
#endif
}  // namespace pose_graph
}  // namespace vtr
