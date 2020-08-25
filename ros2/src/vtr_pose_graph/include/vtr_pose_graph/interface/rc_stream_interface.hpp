#pragma once

#include <stdint.h>
#include <map>
#include <memory>
#include <utility>

#include <vtr_common/utils/lockable.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/interface/rc_interface_types.hpp>

/// #include <robochunk/base/DataBubble.hpp>
#include <vtr_pose_graph/robochunk/base/data_bubble.hpp>

#if 0
#include <stdexcept>
#include <string>
#include <vector>

#include <asrl/messages/Utility.pb.h>
#include <google/protobuf/repeated_field.h>
#include <asrl/common/logging.hpp>
#include <asrl/common/utils/ContainerTools.hpp>

#include <robochunk_msgs/TimeStamp.pb.h>
#include <robochunk/base/ChunkSerializer.hpp>

#endif

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

  using Bubble = robochunk::base::DataBubble;
  using BubblePtr = std::shared_ptr<Bubble>;
  using BubbleMap = std::map<uint32_t, BubblePtr>;
  using LockableBubbleMap = common::Lockable<BubbleMap>;
  using LockableBubbleMapPtr = std::shared_ptr<LockableBubbleMap>;

  // Structures to map between field ids and data streams.
  using StreamPtr = RobochunkIO::StreamPtr;
  using SerializerPtr = RobochunkIO::SerializerPtr;
  using StreamMap = std::map<uint32_t, RobochunkIO>;
  using LockableStreamMap = common::Lockable<StreamMap>;
  using LockableStreamMapPtr = std::shared_ptr<LockableStreamMap>;

  // Stream mutex lock guard
  using Guard = std::unique_lock<std::recursive_mutex>;

  /**
   * \brief default constructor
   */
  RCStreamInterface();

#if 0
  /**
   * \brief construct from messages...
   */
  RCStreamInterface(
      const asrl::graph_msgs::Interval &timeRange,
      const LockableFieldMapPtr &stream_names,
      const LockableStreamMapPtr &streamMap,
      const google::protobuf::RepeatedPtrField<asrl::graph_msgs::IntervalIndex>
          &streamIndices);
  /**
   * \brief copy constructor
   */
  RCStreamInterface(const RCStreamInterface &) = default;

  /**
   * \brief move constructor
   */
  RCStreamInterface(RCStreamInterface &&) = default;

  /**
   * \brief assignment operator
   */
  RCStreamInterface &operator=(const RCStreamInterface &) = default;

  /**
   * \brief assignment operator (move)
   */
  RCStreamInterface &operator=(RCStreamInterface &&) = default;

  /**
   * \brief Serializes the stream index information to the protobuf message.
   * @param timeRange The Time range of this vertex's data bubble.
   * @param streamIndices The repeated protobuf message to serialize the index
   * information
   *                      to.
   */
  void serializeStreams(
      asrl::graph_msgs::Interval *timeRange,
      google::protobuf::RepeatedPtrField<asrl::graph_msgs::IntervalIndex>
          *streamIndices) const;

  /**
   * \brief Sets the time range for this vertex
   * @param time0 The start time.
   * @param time1 The stop time.
   */
  void setTimeRange(uint64_t &time0, uint64_t &time1) {
    timeRange_ = Interval(time0, time1);
  }

  /**
   * \brief Adds indices to a specific stream.
   * \brief stream_name the name of the stream.
   * \brief interval the index interval associated with this stream.
   */
  void addStreamIndices(const std::string &stream_name,
                        const Interval &interval, bool overwrite = false);

  /**
   * \brief Determine if the vertex has an index into a given stream
   */
  inline bool hasStreamIndex(const std::string &stream_name) const {
    return streamNames_->locked().get().count(stream_name);
  }

  /**
   * \brief Loads all of the data associated with this vertex.
   */
  void load();

  /**
   * \brief Unloads all of the data associated with this vertex.
   */
  void unload();

  /**
   * \brief Loads all of the messages associated with this specific stream.
   * @param stream_name the name of the stream
   */
  void load(const std::string &stream_name);

  /**
   * \brief Unloads all of the messages associated with this specific stream.
   * @param stream_name the name of the stream.
   * @throws std_logic error if the stream does not exist
   */
  void unload(const std::string &stream_name);

   * \brief Resets the data bubble in the steam
   * @param the name of the stream the bubble is associated with
  void resetBubble(const std::string &stream_name);

   * \brief resets the stream
   * @param the name of the stream to reset.
  void resetStream(const std::string &stream_name);

  /**
   * \brief Retrieves all of the data associated with the data stream indexed
   *        by the vertex.
   * @param stream_name The name of the stream.
   * @return a vector of pointers to the data entries.
   */
  template <typename MessageType>
  std::vector<std::shared_ptr<MessageType>> retrieveData(
      const std::string &stream_name);

  /**
   * \brief Retrieves a specific message from the data stream, based on
   *        the index.
   * \brief param stream_name the name of the stream.
   * \brief index the index into the data bubble.
   * @note this is a local index, an index of 0 will access the first message
   * in the
   *       bubble.
   */
  template <typename MessageType>
  std::shared_ptr<MessageType> retrieveData(const std::string &stream_name,
                                            uint32_t index);

  /**
   * \brief Retrieves a specific message from the data stream, based on
   *        the time.
   * \brief param stream_name the name of the stream.
   * \brief time The query time stamp.
   * @note This is absolute time. (i.e. nanoseconds since epoch).
   */
  template <typename MessageType>
  std::shared_ptr<MessageType> retrieveData(
      const std::string &stream_name, robochunk::std_msgs::TimeStamp &time);

  /**
   * \brief Retrieves data associated with this vertex's keyframe time.
   * \brief stream_name the name of the stream.
   * @return a shared pointer to the message associated with the keyframe data.
   */
  template <typename MessageType>
  std::shared_ptr<MessageType> retrieveKeyframeData(
      const std::string &stream_name);

  template <typename MessageType>
  bool insert(const std::string &stream_name, const MessageType &message,
              const robochunk::std_msgs::TimeStamp &stamp);

  template <typename MessageType>
  bool insertAndWrite(const std::string &stream_name,
                      const MessageType &message,
                      const robochunk::std_msgs::TimeStamp &stamp);

  void write();
  void write(const std::string &stream_name);
  void write(const uint32_t &stream_idx);

  /**
   * \brief Inserts a Robochunk message
   * \brief stream_name the name of the stream. msg is the message to insert.
   * @return true if success
   */
  bool insert(const std::string &stream_name,
              robochunk::msgs::RobochunkMessage msg);

  /**
   * \brief Sets the stream map that this vertex's run is associated with.
   * @details This map contains pointers to all of the robochunk data streams
   *          associated with the parent run that are responsible for random
   *          access data deserialization.
   * @param The data structure that maps strings to robochunk Data Streams.
   */
  void setStreamMap(LockableStreamMapPtr stream_map) {
    stream_map_ = stream_map;
  }

  void setStreamNameMap(LockableFieldMapPtr stream_name) {
    streamNames_ = stream_name;
  }
#endif

  /// void setKeyFrameTime(const robochunk::std_msgs::TimeStamp &time) {
  ///   keyFrameTime_ = time;
  /// }
  void setKeyFrameTime(const vtr_messages::msg::TimeStamp &time) {
    keyFrameTime_ = time;
  }

  /// const robochunk::std_msgs::TimeStamp &keyFrameTime() {
  ///   return keyFrameTime_;
  /// };
  const vtr_messages::msg::TimeStamp &keyFrameTime() { return keyFrameTime_; };

  bool isDataSaved() { return data_saved_; };

 private:
#if 0
  /**
   * Lock the stream
   */
  struct RWGuard {
    Guard read, write;
  };
  RWGuard lockStream(const FieldMap::mapped_type &stream_idx, bool read = true,
                     bool write = true);
#endif
  bool data_saved_;

  /**
   * \brief Time range associated with this vertex.
   */
  Interval timeRange_;

  /**
   * \brief Data structure that maps stream names to indicies.
   */
  LockableFieldMapPtr streamNames_;

  /**
   * \brief Pointer to the data structure that maps Data Streams to stream
   * names.
   * @details This data structure is owned and instantiated by the parent Run.
   */
  LockableStreamMapPtr stream_map_;

  /**
   * \brief Data structure that maps data indices to streams.
   */
  LockableIntervalMap streamIndices_;

  /**
   * \brief Data structure that maps data bubbles to streams.
   */
  LockableBubbleMapPtr dataBubbleMap_;

  /**
   * \brief The keyframe time associated with this vertex.
   */
  /// robochunk::std_msgs::TimeStamp keyFrameTime_;
  vtr_messages::msg::TimeStamp keyFrameTime_;
};
}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/interface/rc_stream_interface.inl>
