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
 * \file rc_run.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>

namespace vtr {
namespace pose_graph {

template <typename MessageType>
void RCRun::registerVertexStream(const std::string& stream_name,
                                 bool points_to_data,
                                 const RegisterMode& mode) {
  uint32_t stream_index;

  if (hasVertexStream(stream_name)) {
    // in create mode, read/write streams must have been set if stream name
    // exists, so we can simply return.
    if (mode == RegisterMode::Create) return;

    // stream assumed exist
    stream_index = vertexStreamNames_->locked().get().at(stream_name);
  } else {
    // in existing mode, we assume the stream_name exists but the read stream
    // have not been created
    if (mode == RegisterMode::Existing) {
      std::string err{"Trying to load an non-existing stream."};
      CLOG(ERROR, "pose_graph") << err;
      throw std::runtime_error{err};
    }

    {
      auto locked_vertex_stream_names = vertexStreamNames_->locked();
      stream_index = uint32_t(locked_vertex_stream_names.get().size());
      locked_vertex_stream_names.get().emplace(stream_name, stream_index);
    }
    (void)rosbag_streams_->locked().get()[stream_index];
  }

  bool write = (mode == RegisterMode::Create);

  // Only create streams if this is not an ephemeral run and we request it
  if (points_to_data && !isEphemeral()) {
    auto data_directory = fs::path{filePath_}.parent_path() / "sensor_data";
    auto& data_stream = rosbag_streams_->locked().get().at(stream_index);
    // create write stream if not exist
    if (write && !data_stream.second)
      data_stream.second.reset(new storage::DataStreamWriter<MessageType>(
          data_directory, stream_name));
    // create read stream if not exist
    if (!data_stream.first)
      data_stream.first.reset(new storage::DataStreamReader<MessageType>(
          data_directory, stream_name));
  } else {
    LOG(DEBUG) << "Run is ephemeral or does not point to data; not "
                  "initializing streams for "
               << stream_name;
  }
}

template <class G>
size_t RCRun::loadHeaderInternal(const std::string& fpath) {
  storage::DataStreamReader<typename G::HeaderMsg> reader{fs::path{fpath} /
                                                          "header"};
  auto msg = reader.readAtIndex(1);
  return size_t(msg->template get<typename G::HeaderMsg>().type.type);
}

template <class M1, class M2>
size_t RCRun::loadDataInternal(M1& dataMap, M2& dataMapInternal,
                               LockableFieldMapPtr& streamNames,
                               const std::string& fpath,
                               const std::unordered_set<IdType>& runs) {
  using G = typename M1::mapped_type::element_type;  // Get the actual graph
                                                     // object type
  typename G::HeaderMsg head;
  typename G::Msg msg;

  if (!fs::exists(fs::path{fpath})) {
    std::stringstream err;
    err << "File " << fpath << " does not exist!";
    CLOG(ERROR, "pose_graph") << err.str();
    throw std::runtime_error{err.str()};
  }

  storage::DataStreamReader<typename G::HeaderMsg> header_reader{
      fs::path{fpath} / "header"};
  try {
    auto msg = header_reader.readAtIndex(1);
    if (!msg) {
      std::stringstream err;
      err << "Failed to deserialize " << fpath << " for run " << id_;
      CLOG(ERROR, "pose_graph") << err.str();
      throw std::runtime_error{err.str()};
    }
    head = msg->template get<typename G::HeaderMsg>();
  } catch (std::exception& e) {
    std::stringstream err;
    err << "Deserializing " << fpath << " for run " << id_
        << ": Error: " << e.what();
    CLOG(ERROR, "pose_graph") << err.str();
    throw std::runtime_error(err.str());
  }

  if (head.run_id != id_) {
    std::stringstream err;
    err << "The graph index is broken; run " << id_ << " points to file "
        << fpath << " with run id " << head.run_id;
    CLOG(ERROR, "pose_graph") << err.str();
    throw std::runtime_error(err.str());
  }

  for (unsigned i = 0; i < head.stream_names.size(); ++i) {
    streamNames->locked().get().emplace(head.stream_names[i], i);
    auto locked_rosbag_streams = rosbag_streams_->locked();
    (void)locked_rosbag_streams.get()[i];
  }

  /// \note this is not used anymore (remove this when cleaning up pose graph)
  wasLoaded_ = true;

  CLOG(DEBUG, "pose_graph") << "Loading data header - run id: " << head.run_id
                            << ", stream names: " << head.stream_names;

  CLOG(DEBUG, "pose_graph") << "Loading data content.";
  size_t last_idx = 0;
  storage::DataStreamReader<typename G::Msg> reader{fs::path{fpath} /
                                                    "content"};
  for (int load_idx = 1;; load_idx++) {
    try {
      auto loaded_msg = reader.readAtIndex(load_idx);
      if (!loaded_msg) {
        // no more indices
        break;
      }
      auto msg = loaded_msg->template get<typename G::Msg>();

      CLOG(DEBUG, "pose_graph") << "Loading message with id: " << msg.id;

      if (msg.id > last_idx) last_idx = msg.id;

      auto new_ptr = G::MakeShared(msg, id_, streamNames, rosbag_streams_);
      dataMapInternal.insert(std::make_pair(new_ptr->id(), new_ptr));

      if (runs.size() == 0 || G::MeetsFilter(msg, runs))
        dataMap.insert(std::make_pair(new_ptr->simpleId(), new_ptr));

    } catch (storage::NoBagExistsException& e) {
#if false
      std::stringstream err;
      err << "Exception occurred at loadDataInternal. Index: " << load_idx
          << ". Error: no bag exists exception, " << e.what();
      CLOG(ERROR, "pose_graph") << err.str();
      throw std::runtime_error(err.str());
#endif
      CLOG(WARNING, "pose_graph")
          << "No bag found when loading from " << fpath << "/content. "
          << "This could be due to no stream being stored to edges of this "
             "type or vertices.";
      break;
    } catch (std::exception& e) {  /// \todo (yuchen) what to catch?
      std::stringstream err;
      err << "Exception occurred at loadDataInternal. Index: " << load_idx
          << ". Error: " << e.what();
      CLOG(ERROR, "pose_graph") << err.str();
      throw std::runtime_error(err.str());
    }
  }
  return last_idx;
}

template <class G>
typename G::HeaderMsg RCRun::populateHeader(const LockableFieldMapPtr& fields,
                                            const G& example) {
  typename G::HeaderMsg header;
  header.run_id = id_;

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
      header.stream_names.push_back(it.second);
    }
  }

  populateHeaderEnum(header, example.id());

  CLOG(DEBUG, "pose_graph") << "Saving data header - run id: " << id_
                            << ", stream names: " << header.stream_names;

  return header;
}

template <class M>
void RCRun::populateHeaderEnum(M&, const BaseId&) {}

template <class M>
void RCRun::populateHeaderEnum(M& msg, const typename EdgeIdType::Base& id) {
  msg.type.type = (unsigned)id.idx();
}

template <class M>
void RCRun::saveDataInternal(M& dataMap, LockableFieldMapPtr& streamNames,
                             const std::string& fpath) {
  using G = typename M::mapped_type::element_type;  // Get the actual graph
                                                    // object type

  if (dataMap.size() > 0) {
    auto head = populateHeader(streamNames, *(dataMap.begin()->second));
    storage::DataStreamWriter<typename G::HeaderMsg> header_writer{
        fs::path{fpath} / "header"};
    header_writer.write(head);

    CLOG(DEBUG, "pose_graph") << "Saving data content.";
    storage::DataStreamWriter<typename G::Msg> writer{fs::path{fpath} /
                                                      "content"};
    for (auto it = dataMap.begin(); it != dataMap.end(); ++it) {
      auto msg = it->second->toRosMsg();
      writer.write(msg);
    }
  } else {
    auto head = populateHeader(streamNames, G());
    storage::DataStreamWriter<typename G::HeaderMsg> header_writer{
        fs::path{fpath} / "header"};
    header_writer.write(head);
  }
}
#if 0
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
