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
 * \file rc_run.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <filesystem>

#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {

RCRun::RCRun()
    : RunBase<RCVertex, RCEdge>(),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      edgeStreamNames_(LockableFieldMapPtrArray()),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(""),
      msg_(),
      readOnly_(false),
      wasLoaded_(false) {
  for (auto it = edgeStreamNames_.begin(); it != edgeStreamNames_.end(); ++it)
    *it = LockableFieldMapPtr(new LockableFieldMap());
}

RCRun::RCRun(const IdType& runId, const IdType& graphId)
    : RunBase<RCVertex, RCEdge>(runId, graphId),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      edgeStreamNames_(LockableFieldMapPtrArray()),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(""),
      msg_(),
      readOnly_(true),
      wasLoaded_(false) {
#if 0
  /// \todo used to not have this
  for (auto it = edgeStreamNames_.begin(); it != edgeStreamNames_.end(); ++it)
    *it = LockableFieldMapPtr(new LockableFieldMap());
#endif
  msg_.id = runId;
  msg_.graph_id = graphId;
}

RCRun::RCRun(const std::string& filePath, const IdType& runId,
             const IdType& graphId)
    : RunBase<RCVertex, RCEdge>(runId, graphId),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      edgeStreamNames_(LockableFieldMapPtrArray()),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(filePath),
      msg_(),
      readOnly_(false),
      wasLoaded_(false) {
  for (auto it = edgeStreamNames_.begin(); it != edgeStreamNames_.end(); ++it)
    *it = LockableFieldMapPtr(new LockableFieldMap());

  msg_.graph_id = graphId_;
  msg_.id = id_;
  msg_.vertex_rpath = "vertex";
  msg_.edge_rpaths.push_back("temporal_edge");
  msg_.edge_rpaths.push_back("spatial_edge");
}

RCRun::RCRun(const std::string& filePath)
    : RunBase<RCVertex, RCEdge>(),
      vertexStreamNames_(LockableFieldMapPtr(new LockableFieldMap())),
      edgeStreamNames_(LockableFieldMapPtrArray()),
      rosbag_streams_(LockableDataStreamMapPtr(new LockableDataStreamMap())),
      filePath_(filePath),
      msg_(),
      readOnly_(true),
      wasLoaded_(false),
      robotId_(-1) {
  loadIndex();
}

void RCRun::load(VertexPtrMapExtern& vertexDataMap,
                 EdgePtrMapExtern& edgeDataMap, const RunFilter& runFilter) {
  if (isEphemeral()) return;

  bool calc_manual = loadIndex();
  loadVertices(vertexDataMap);
  loadEdges(edgeDataMap, runFilter);
#if 0
  /// \todo (yuchen) not sure if this is needed.
  if (msg_.reindex) {
    for (auto&& it :
         common::utils::getRefs(vertexStreamNames_->locked().get())) {
      reindexStream(it.get().first, WindowType::Before, false);
    }

    msg_.reindex = false;
  }
#endif

  if (calc_manual) computeManual();
}

bool RCRun::loadIndex() {
  if (isEphemeral()) return false;

  storage::DataStreamReader<vtr_messages::msg::GraphRun> reader{
      fs::path{filePath_} / "run_index"};
  auto msg_ptr = reader.readAtIndex(1);
  msg_ = msg_ptr->get<vtr_messages::msg::GraphRun>();

  id_ = msg_.id;
  graphId_ = msg_.graph_id;
  robotId_ = msg_.robot_id;
  /// \todo we assume that the contains manual flag always presents and is
  /// always valid. But we ALWAYS COMPUTE MANUAL in load function.
  manual_ = msg_.contains_manual;

  std::stringstream ss;
  ss << "Loading run index from message" << std::endl;
  ss << "- run id (not used right now): " << msg_.id << std::endl;
  ss << "- graph id (not used right now): " << msg_.graph_id << std::endl;
  ss << "- robot id (not used right now): " << msg_.robot_id << std::endl;
  ss << "- contains_manual: " << msg_.contains_manual << std::endl;
  ss << "- vertex relative path: " << msg_.vertex_rpath << std::endl;
  ss << "- edge relative paths: " << std::endl;
  for (const auto& p : msg_.edge_rpaths) ss << "  - " << p << std::endl;
  CLOG(DEBUG, "pose_graph") << ss.str();

  readOnly_ = true;

  return true;
}

void RCRun::loadVertices(VertexPtrMapExtern& vertexDataMap) {
  if (isEphemeral()) return;

  CLOG(DEBUG, "pose_graph") << "Loading vertices from message.";
  loadDataInternal(vertexDataMap, vertices_, vertexStreamNames_,
                   fs::path{filePath_} / msg_.vertex_rpath);
  readOnly_ = true;
}

void RCRun::loadEdges(EdgePtrMapExtern& edgeDataMap,
                      const RunFilter& runFilter) {
  if (isEphemeral()) return;

  for (unsigned i = 0; i < msg_.edge_rpaths.size(); ++i) {
    CLOG(DEBUG, "pose_graph")
        << "Loading edges of type " << i << " from message.";
    auto idx =
        loadHeaderInternal<RCEdge>(fs::path{filePath_} / msg_.edge_rpaths[i]);

    auto num =
        loadDataInternal(edgeDataMap, edges_[idx], edgeStreamNames_[idx],
                         fs::path{filePath_} / msg_.edge_rpaths[i], runFilter);

    if (idx >= EdgeIdType::NumTypes() && num > 0) {
      throw std::invalid_argument(
          "These edge objects don't recognize the edge message types you are "
          "trying to load.");
    }
  }

  readOnly_ = true;
}

#if 0
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

void RCRun::save(bool force) {
  CLOG(DEBUG, "pose_graph") << "Saving run of id " << id();

  if (isEphemeral() || (readOnly_ && !force)) return;

  saveIndex(true);
  saveVertices(true);
  saveEdges(true);
}

void RCRun::saveIndex(bool force) {
  if (isEphemeral() || (readOnly_ && !force)) return;

  msg_.contains_manual = manual_;

  std::stringstream ss;
  ss << "Saving run index to message" << std::endl;
  ss << "- run id (not used right now): " << msg_.id << std::endl;
  ss << "- graph id (not used right now): " << msg_.graph_id << std::endl;
  ss << "- robot id (not used right now): " << msg_.robot_id << std::endl;
  ss << "- contains_manual: " << msg_.contains_manual << std::endl;
  ss << "- vertex relative path: " << msg_.vertex_rpath << std::endl;
  ss << "- edge relative paths: " << std::endl;
  for (const auto& p : msg_.edge_rpaths) ss << "  - " << p << std::endl;
  CLOG(DEBUG, "pose_graph") << ss.str();

  storage::DataStreamWriter<vtr_messages::msg::GraphRun> writer{
      fs::path{filePath_} / "run_index"};
  writer.write(msg_);

  readOnly_ = true;
}

void RCRun::saveVertices(bool force) {
  if (isEphemeral() || (readOnly_ && !force)) return;

  CLOG(DEBUG, "pose_graph") << "Saving vertices to message.";
  saveDataInternal(vertices_, vertexStreamNames_,
                   fs::path{filePath_} / msg_.vertex_rpath);

  readOnly_ = true;
}

void RCRun::saveEdges(bool force) {
  if (isEphemeral() || (readOnly_ && !force)) return;

  for (unsigned int i = 0; i < edges_.size(); ++i) {
    CLOG(DEBUG, "pose_graph") << "Saving edges of type " << i << " to message.";
    saveDataInternal(edges_[i], edgeStreamNames_[i],
                     fs::path{filePath_} / msg_.edge_rpaths[i]);
  }

  readOnly_ = true;
}

#if 0
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

#if 0
void RCRun::setFilePath(const std::string& fpath) {
  if (wasLoaded_) {
    throw std::runtime_error(
        "You cannot change the index file after loading data!");
  }

  filePath_ = fpath;
}

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
  if (isEphemeral()) return;

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
  if (isEphemeral()) return;

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
  /// vertex->setStreamMap(robochunkStreams_);
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
