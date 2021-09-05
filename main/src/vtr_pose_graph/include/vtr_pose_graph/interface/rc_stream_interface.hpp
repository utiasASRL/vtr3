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
 * \file rc_stream_interface.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <stdint.h>
#include <map>
#include <memory>
#include <utility>

#include <vtr_common/utils/lockable.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_messages/msg/util_interval.hpp>
#include <vtr_messages/msg/util_interval_named.hpp>
#include <vtr_pose_graph/interface/rc_interface_types.hpp>
#include <vtr_storage/stream/data_bubble.hpp>

namespace vtr {
namespace pose_graph {
class RCStreamInterface {
 public:
  using Interval = std::pair<uint64_t, uint64_t>;
  using IntervalMap = std::map<uint32_t, Interval>;
  using LockableIntervalMap = common::Lockable<IntervalMap>;

  using FieldMap = std::map<std::string, uint32_t>;
  using LockableFieldMap = common::Lockable<FieldMap>;
  using LockableFieldMapPtr = std::shared_ptr<LockableFieldMap>;

  // Structures to map between field ids and data streams. (rosbag2)
  using DataStreamMap = std::map<uint32_t, RosBagIO>;
  using LockableDataStreamMap = common::Lockable<DataStreamMap>;
  using LockableDataStreamMapPtr = std::shared_ptr<LockableDataStreamMap>;

  using DataBubble = storage::DataBubble;
  using DataBubblePtr = std::shared_ptr<DataBubble>;
  using DataBubbleMap = std::map<uint32_t, DataBubblePtr>;
  using LockableDataBubbleMap = common::Lockable<DataBubbleMap>;
  using LockableDataBubbleMapPtr = std::shared_ptr<LockableDataBubbleMap>;

  // Stream mutex lock guard
  using Guard = std::unique_lock<std::recursive_mutex>;

  RCStreamInterface();
  /** \brief construct from messages... */
  RCStreamInterface(
      const vtr_messages::msg::UtilInterval &time_range,
      const LockableFieldMapPtr &stream_names,
      const LockableDataStreamMapPtr &stream_map,
      const std::vector<vtr_messages::msg::UtilIntervalNamed> &stream_indices);
#if 0
  RCStreamInterface(const RCStreamInterface &) = default;
  RCStreamInterface(RCStreamInterface &&) = default;

  RCStreamInterface &operator=(const RCStreamInterface &) = default;
  RCStreamInterface &operator=(RCStreamInterface &&) = default;
#endif
  /**
   * \brief Serializes the stream index information to ros message.
   * \return {time_range, stream_indices} The Time range of this vertex's data
   * bubble, and repeated ros message to serialize the index information to.
   */
  std::tuple<vtr_messages::msg::UtilInterval,
             std::vector<vtr_messages::msg::UtilIntervalNamed>>
  serializeStreams() const;

  /**
   * \brief Sets the time range for this vertex
   * \todo we may be able to remove this function as it is only used in tests
   * \param time0 The start time.
   * \param time1 The stop time.
   */
  void setTimeRange(uint64_t &time0, uint64_t &time1) {
    time_range_ = Interval(time0, time1);
  }

  /**
   * \brief Adds indices to a specific stream.
   * \todo This function also seems only used in tests. check removable?
   * \param stream_name the name of the stream.
   * \param interval the index interval associated with this stream.
   * \param overwrite whether or not to overwrite existing index
   */
  template <typename MessageType>
  void addStreamIndices(const std::string &stream_name,
                        const Interval &interval, bool overwrite = false);

#if 0
  /** \brief Determine if the vertex has an index into a given stream */
  inline bool hasStreamIndex(const std::string &stream_name) const {
    return stream_names_->locked().get().count(stream_name);
  }
#endif

  /** \brief Loads all of the data associated with this vertex. */
  void load();

  /**
   * \brief Loads all of the messages associated with this specific stream.
   * \param stream_name the name of the stream
   */
  void load(const std::string &stream_name);

  /** \brief Unloads all of the data associated with this vertex. */
  void unload();
#if 0
  /**
   * \brief Unloads all of the messages associated with this specific stream.
   * \param stream_name the name of the stream.
   * \throw std_logic error if the stream does not exist
   */
  void unload(const std::string &stream_name);

  /**
   * \brief Resets the data bubble in the steam
   * \param the name of the stream the bubble is associated with
   */
  void resetBubble(const std::string &stream_name);

  /**
   * \brief Retrieves all of the data associated with the data stream indexed
   *        by the vertex.
   * \param stream_name The name of the stream.
   * \return a vector of pointers to the data entries.
   */
  template <typename MessageType>
  std::vector<std::shared_ptr<MessageType>> retrieveData(
      const std::string &stream_name);
#endif
  /**
   * \brief Retrieves a specific message from the data stream, based on index
   * \param stream_name the name of the stream.
   * \param index the index into the data bubble.
   * \param allow_nullptr allow returning nullptr if failed.
   * \note this is a local index, an index of 0 will access the first message
   * in the bubble.
   */
  template <typename MessageType>
  std::shared_ptr<MessageType> retrieveData(const std::string &stream_name,
                                            uint32_t index,
                                            bool allow_nullptr = false);

  /**
   * \brief Retrieves a specific message from the data stream, based on time
   * \param stream_name the name of the stream.
   * \param time The query time stamp.
   * \param allow_nullptr allow returning nullptr if failed.
   * \note This is absolute time. (i.e. nanoseconds since epoch).
   */
  template <typename MessageType>
  std::shared_ptr<MessageType> retrieveData(const std::string &stream_name,
                                            vtr_messages::msg::TimeStamp &time,
                                            bool allow_nullptr = false);

  /**
   * \brief Retrieves data associated with this vertex's keyframe time.
   * \param stream_name the name of the stream.
   * \param allow_nullptr allow returning nullptr if failed.
   * \return a shared pointer to the message associated with the keyframe data.
   */
  template <typename MessageType>
  std::shared_ptr<MessageType> retrieveKeyframeData(
      const std::string &stream_name, bool allow_nullptr = false) {
    return retrieveData<MessageType>(stream_name, keyframe_time_,
                                     allow_nullptr);
  }

  void write();
  void write(const std::string &stream_name);
  void write(const uint32_t &stream_idx);

#if 0
  template <typename MessageType>
  bool insertAndWrite(const std::string &stream_name,
                      const MessageType &message,
                      const robochunk::std_msgs::TimeStamp &stamp);
#endif

  template <typename MessageType>
  bool insert(const std::string &stream_name, MessageType &message,
              const vtr_messages::msg::TimeStamp &stamp) {
    // set message time stamp and insert the data
    storage::VTRMessage vtr_msg{message};
    vtr_msg.set_timestamp(stamp.nanoseconds_since_epoch);

    // insert into the vertex
    return insert(stream_name, vtr_msg);
  }

  template <typename MessageType>
  bool replace(const std::string &stream_name, MessageType &message,
               const vtr_messages::msg::TimeStamp &stamp) {
    // set message time stamp and insert the data
    storage::VTRMessage vtr_msg{message};
    vtr_msg.set_timestamp(stamp.nanoseconds_since_epoch);

    // insert into the vertex
    return replace(stream_name, vtr_msg);
  }

  /**
   * \brief Inserts a message to the data bubble.
   * \param stream_name the name of the stream.
   * \param vtr_msg the message to insert.
   * \return true if success
   */
  bool insert(const std::string &stream_name, storage::VTRMessage &vtr_msg);

  /**
   * \brief Resets the data bubble and inserts a message.
   * \param stream_name the name of the stream.
   * \param vtr_msg the message to replace.
   * \return true if success
   */
  bool replace(const std::string &stream_name, storage::VTRMessage &vtr_msg);

  /**
   * \brief Sets the stream map that this vertex's run is associated with.
   * \details This map contains pointers to all of the robochunk data streams
   *          associated with the parent run that are responsible for random
   *          access data deserialization.
   * \param data_stream_map The data structure that maps strings to robochunk
   * Data Streams.
   */
  void setDataStreamMap(LockableDataStreamMapPtr data_stream_map) {
    data_stream_map_ = data_stream_map;
  }

  void setStreamNameMap(LockableFieldMapPtr stream_name) {
    stream_names_ = stream_name;
  }

  void setKeyFrameTime(const vtr_messages::msg::TimeStamp &time) {
    keyframe_time_ = time;
  }

  const vtr_messages::msg::TimeStamp &keyFrameTime() { return keyframe_time_; };

  bool isDataSaved() { return data_saved_; };

 private:
  /**
   * \brief Get the data bubble associated with the stream name.
   * \param[in] stream_name the name of the stream.
   * \param[out] stream_idx the idx of the stream
   * \return a shared pointer to the data bubble or nullptr.
   */
  DataBubblePtr getDataBubbleAndStreamIndex(const std::string &stream_name,
                                            FieldMap::mapped_type &stream_idx);

  /** Lock the stream */
  struct RWGuard {
    Guard read, write;
  };
  RWGuard lockReadWriteStream(const FieldMap::mapped_type &stream_idx,
                              bool read = true, bool write = true);

  /**
   * \brief Whether there are unsaved data in the bubble
   * \note For now, access to this variable must also be protected by the data
   * bubble map lock
   */
  bool data_saved_;

  /**
   * \brief Map from stream indices to data indices (start, end).
   * \note Given that we use time range most of the time. This structure may not
   * be very useful.
   * \note For now, access to this data must also be protected by the data
   * bubble map lock
   */
  LockableIntervalMap stream_indices_;

  /** \brief Time range associated with this vertex for all data. */
  Interval time_range_;

  /** \brief The keyframe time associated with this vertex. */
  vtr_messages::msg::TimeStamp keyframe_time_;

  /** \brief Map from stream name to stream index. */
  LockableFieldMapPtr stream_names_;

  /** \brief Map from stream index to data stream for disk IO. */
  LockableDataStreamMapPtr data_stream_map_;

  /**
   * \brief Map from stream index to data bubble for caching.
   * \note \todo currently data bubble only has read stream so it does not have
   * an api to store data into disk. Any reason why we do not put the write
   * stream into the data bubble as well?
   */
  LockableDataBubbleMapPtr data_bubble_map_;
};
}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/interface/rc_stream_interface.inl>
