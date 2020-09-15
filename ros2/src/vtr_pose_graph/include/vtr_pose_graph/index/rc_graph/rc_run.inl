#pragma once

#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>

namespace vtr {
namespace pose_graph {
template <typename MessageType>
void RCRun::registerVertexStream(const std::string& stream_name,
                                 bool points_to_data,
                                 const RegisterMode& mode) {
  uint32_t stream_index;

  // We already have the stream
  if (hasVertexStream(stream_name)) {
    if (mode != RegisterMode::Replace) {
      LOG(DEBUG) << "Stream " << stream_name
                 << " exists and no overwrite was requested";
      return;
    }

    stream_index = vertexStreamNames_->locked().get().at(stream_name);
    LOG(WARNING) << "Overwriting data for stream " << stream_name
                 << " (I hope you're sure...)";
    // Reset data bubble and indices maps from within each vertex.
    for (auto&& it : vertices_) it.second->resetStream(stream_name);

  } else {
    {
      auto locked_vertex_stream_names = vertexStreamNames_->locked();
      stream_index = uint32_t(locked_vertex_stream_names.get().size());
      locked_vertex_stream_names.get().emplace(stream_name, stream_index);
    }
#if 1
    (void)robochunkStreams_->locked().get()[stream_index];
#endif
    (void)rosbag_streams_->locked().get()[stream_index];
  }
#if 0
  using namespace robochunk::base;
  uint32_t max_file_size_GB = 5;
#endif
  bool overwrite =
      (mode == RegisterMode::Replace) || (mode == RegisterMode::Create);
  // Only create streams if this is not an ephemeral run and we request it
  if (points_to_data && !isEphemeral()) {
    /// auto data_directory = robochunk::util::split_directory(
    ///     robochunk::util::split_directory(filePath_));
    fs::path file_path{filePath_};
    auto data_directory = file_path.parent_path().parent_path();
    if (overwrite || (mode == RegisterMode::Append)) {
#if 1
      /// robochunkStreams_->locked().get().at(stream_index).second =
      ///     SerializerFactory::createSerializer(data_directory, "/" +
      ///     stream_name, overwrite, max_file_size_GB);
      robochunkStreams_->locked()
          .get()
          .at(stream_index)
          .second.reset(new robochunk::base::ChunkSerializer(
              std::string{data_directory}, stream_name));
#endif
      rosbag_streams_->locked()
          .get()
          .at(stream_index)
          .second.reset(new storage::DataStreamWriter<MessageType>(
              data_directory, stream_name));
    } else {
      LOG(DEBUG) << "Run was read only; not initializing serializer for stream "
                 << stream_name;
    }
#if 1
    /// robochunkStreams_->locked()
    ///     .get()
    ///     .at(stream_index)
    ///     .first.reset(new ChunkStream(data_directory, "/" + stream_name));
    robochunkStreams_->locked()
        .get()
        .at(stream_index)
        .first.reset(
            new robochunk::base::ChunkStream(data_directory, stream_name));
#endif
    rosbag_streams_->locked()
        .get()
        .at(stream_index)
        .first.reset(new storage::DataStreamReader<MessageType>(data_directory,
                                                                stream_name));
  } else {
    LOG(DEBUG) << "Run is ephemeral or does not point to data; not "
                  "initializing streams for "
               << stream_name;
  }
}
#if 0
template <class M1, class M2>
size_t RCRun::loadDataInternal(M1& dataMap, M2& dataMapInternal,
                               LockableFieldMapPtr& streamNames,
                               const std::string& fpath,
                               const std::unordered_set<IdType>& runs) {
  typedef typename M1::mapped_type::element_type
      G;  // Get the actual graph object type
  typename G::HeaderMsg head;
  typename G::Msg msg;
  size_t lastIdx = 0;

  if (robochunk::util::file_exists(fpath) == false) {
    LOG(ERROR) << "File " << fpath << " does not exist!";
    return 0;
  }
  robochunk::base::DataInputStream istream;
  istream.openStream(fpath);
  bool success = istream.deserialize(head);
  if (success == false) {
    LOG(ERROR) << "Failed to deserialize " << fpath << " for run" << id_;
    return 0;
  };

  if (head.runid() != id_) {
    istream.closeStream();
    std::stringstream ss("The graph index is broken; run ");
    ss << id_ << " points to file " << fpath << " with run id " << head.runid();
    throw std::runtime_error(ss.str());
  }

  wasLoaded_ = true;

  auto data_directory = robochunk::util::split_directory(
      robochunk::util::split_directory(filePath_));
  for (int i = 0; i < head.streamnames_size(); ++i) {
    streamNames->locked().get().emplace(head.streamnames(i), i);
    auto locked_robochunk_streams = robochunkStreams_->locked();
    auto& new_stream = locked_robochunk_streams.get()[i];
    if (robochunk::util::file_exists(data_directory + "/sensorData/" +
                                     head.streamnames(i))) {
      // Set up the robochunk stream to open the files at the stream name.
      new_stream.first = std::make_shared<robochunk::base::ChunkStream>(
          data_directory, "/" + head.streamnames(i));
    }
    //    else {
    //      LOG(DEBUG) << "Data for stream " << head.streamnames(i) << " was not
    //      found in " << data_directory;
    //    }
  }
  unsigned int nbytes = istream.fileSizeBytes();

  while (istream.streamPosition() < nbytes) {
    istream.deserialize(msg);
    if (msg.id() > lastIdx) {
      lastIdx = msg.id();
    }

    typename G::Ptr newPtr =
        G::MakeShared(msg, id_, streamNames, robochunkStreams_);
    dataMapInternal.insert(std::make_pair(newPtr->id(), newPtr));

    if (runs.size() == 0 || G::MeetsFilter(msg, runs)) {
      dataMap.insert(std::make_pair(newPtr->simpleId(), newPtr));
    }
  }

  istream.closeStream();
  return lastIdx;
}

template <class G>
typename G::HeaderMsg RCRun::populateHeader(const LockableFieldMapPtr& fields,
                                            const G& example) {
  typename G::HeaderMsg header;
  header.set_runid(id_);

  if (fields != nullptr) {
    std::map<uint32_t, std::string> inverseStreamNames;

    // Lock the fields while we iterate through
    {
      auto locked_fields = fields->locked();
      for (auto&& it : locked_fields.get()) {
        inverseStreamNames.emplace(it.second, it.first);
      }
    }

    for (auto&& it : inverseStreamNames) {
      header.add_streamnames(it.second);
    }
  }

  populateHeaderEnum(header, example.id());
  return header;
}

template <class M>
void RCRun::populateHeaderEnum(M&, const BaseId&) {}

template <class M>
void RCRun::populateHeaderEnum(M& msg, const typename EdgeIdType::Base& id) {
  msg.set_type(asrl::graph_msgs::EdgeType(id.idx()));
}

template <class M>
void RCRun::saveDataInternal(M& dataMap, LockableFieldMapPtr& streamNames,
                             const std::string& fpath) {
  typedef typename M::mapped_type::element_type
      G;  // Get the actual graph object type
  typename G::Msg msg;
  robochunk::base::DataOutputStream ostream;

  if (robochunk::util::file_exists(fpath)) {
    if (robochunk::util::file_exists(fpath + ".tmp")) {
      std::remove((fpath + ".tmp").c_str());
    }
    robochunk::util::move_file(fpath, fpath + ".tmp");
  }

  ostream.openStream(fpath, true);

  if (dataMap.size() > 0) {
    auto head = populateHeader(streamNames, *(dataMap.begin()->second));
    ostream.serialize(head);

    for (auto it = dataMap.begin(); it != dataMap.end(); ++it) {
      it->second->toProtobuf(&msg);
      ostream.serialize(msg);
    }
  } else {
    auto head = populateHeader(streamNames, G());
    ostream.serialize(head);
  }

  ostream.closeStream();

  if (robochunk::util::file_exists(fpath + ".tmp")) {
    std::remove((fpath + ".tmp").c_str());
  }
}

template <class G>
std::string RCRun::workingFile(const G& obj, const std::string& basePath) {
  std::stringstream tmp;
  tmp << basePath + "/working/" + obj.name() + "_";
  tmp << std::setfill('0') << std::setw(9) << id_ << "_";
  tmp << std::setfill('0') << std::setw(9) << obj.id().minorId() << ".proto";

  return tmp.str();
}

template <class M>
void RCRun::saveWorkingInternal(M& dataMap,
                                const LockableFieldMapPtr& streamNames,
                                const std::string& basePath) {
  typedef typename M::mapped_type::element_type
      G;  // Get the actual graph object type
  typename G::Msg msg;

  robochunk::base::DataOutputStream ostream;

  // Write a temporary vertex/edge header to preserve stream name changes
  if (dataMap.size() > 0) {
    auto head = populateHeader(streamNames, *(dataMap.begin()->second));

    ostream.openStream(
        basePath + "/working/" + dataMap.begin()->second->name() + ".proto",
        true);
    ostream.serialize(head);
    ostream.closeStream();
  }

  for (auto it = dataMap.begin(); it != dataMap.end(); ++it) {
    if (it->second->isModified()) {
      it->second->toProtobuf(&msg);
      ostream.openStream(workingFile(*it->second, basePath), true);
      ostream.serialize(msg);
      ostream.closeStream();
    }
  }
}

template <typename MessageType>
bool RCRun::insert(const std::string& stream_name, const MessageType& message,
                   const robochunk::std_msgs::TimeStamp& stamp) {
  // Convert the message to a RobochunkMessage
  robochunk::msgs::RobochunkMessage msg;
  msg.mutable_header()
      ->mutable_sensor_time_stamp()
      ->set_nanoseconds_since_epoch(stamp.nanoseconds_since_epoch());
  msg.setPayload(message);

  // Push the message onto the stream queue.
  // A new queue is automatically created if none exists for the stream_name

  std::lock_guard<std::mutex> lock(streamBufferLock_);
  streamBuffers_[stream_name].push(msg);

  return true;
}
#endif
}  // namespace pose_graph
}  // namespace vtr
