#include <vtr_common/utils/container_tools.hpp>
#include <vtr_pose_graph/interface/rc_stream_interface.hpp>

#if 0
#include <asrl/common/timing/SimpleTimer.hpp>
#endif

namespace vtr {
namespace pose_graph {

RCStreamInterface::RCStreamInterface()
    : data_saved_(false),
      timeRange_(Interval()),
      streamNames_(LockableFieldMapPtr()),
      data_stream_map_(LockableDataStreamMapPtr()),
      streamIndices_(IntervalMap()),
      data_bubble_map_(new LockableDataBubbleMap()) {
}

RCStreamInterface::RCStreamInterface(
    const vtr_messages::msg::UtilInterval &timeRange,
    const LockableFieldMapPtr &stream_names,
    const LockableDataStreamMapPtr &data_stream_map,
    const std::vector<vtr_messages::msg::UtilIntervalNamed> &streamIndices)
    : data_saved_(false),
      timeRange_(Interval(timeRange.idx1, timeRange.idx2)),
      streamNames_(stream_names),
      data_stream_map_(data_stream_map),
      streamIndices_(IntervalMap()) {
  keyFrameTime_.nanoseconds_since_epoch = timeRange.idx2;

  data_bubble_map_.reset(new LockableDataBubbleMap());
  for (auto it = streamIndices.begin(); it != streamIndices.end(); ++it) {
    streamIndices_.locked().get().emplace(it->name_idx,
                                          Interval(it->idx1, it->idx2));

    // add to the data bubble
    // auto &rosbag_io = data_stream_map_->locked().get().at(it->name_idx);
    // \todo (yuchen) we have to create the data bubbles here because
    // streamIndices
    // if (rosbag_io.first != nullptr) {
    auto data_bubble =
        data_bubble_map_->locked()
            .get()
            .emplace(it->name_idx, std::make_shared<DataBubble>())
            .first->second;
    // \todo (yuchen) cannot initialize data bubble until type is known
    // We need to figure out a better way to find data type...
    // data_bubble->initialize(rosbag_io.first);
    data_bubble->setIndices(it->idx1, it->idx2);
    // }
  }
}

std::tuple<vtr_messages::msg::UtilInterval,
           std::vector<vtr_messages::msg::UtilIntervalNamed>>
RCStreamInterface::serializeStreams() const {
  vtr_messages::msg::UtilInterval time_range;
  time_range.idx1 = timeRange_.first;
  time_range.idx2 = timeRange_.second;

  std::vector<vtr_messages::msg::UtilIntervalNamed> stream_indices;
  stream_indices.reserve(streamIndices_.locked().get().size());
  for (auto &&it : common::utils::getRefs(streamIndices_.locked().get())) {
    vtr_messages::msg::UtilIntervalNamed tmp_msg;
    tmp_msg.name_idx = it.get().first;
    tmp_msg.idx1 = it.get().second.first;
    tmp_msg.idx2 = it.get().second.second;
    stream_indices.push_back(tmp_msg);
  }

  return {time_range, stream_indices};
}

void RCStreamInterface::load() {
  for (auto &&itr : common::utils::getRefs(data_bubble_map_->locked().get()))
    itr.get().second->load();
}

void RCStreamInterface::load(const std::string &stream_name) {
  FieldMap::mapped_type stream_idx;
  {
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(stream_name);
    if (stream_itr == locked_stream_names.get().end())
      return;
    stream_idx = stream_itr->second;
  }

  DataBubbleMap::mapped_type data_bubble;
  {
    auto locked_data_bubble_map = data_bubble_map_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);
    if (bubble_itr == locked_data_bubble_map.get().end())
      return;
    data_bubble = bubble_itr->second;
  }

  // this assumes that we have created the stream reader
  if (!data_bubble->isInitialized())
    data_bubble->initialize(
        data_stream_map_->locked().get().at(stream_idx).first);

  // grab the mutex from the stream map
  auto guard = lockStream(stream_idx, true, false);
  data_bubble->load();
}

void RCStreamInterface::unload() {
  for (auto &&itr : common::utils::getRefs(data_bubble_map_->locked().get()))
    itr.get().second->unload();
}

#if 0
void RCStreamInterface::unload(const std::string &stream_name) {
  FieldMap::mapped_type stream_idx;
  {
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(stream_name);
    if (stream_itr == locked_stream_names.get().end()) {
      return;
    }
    stream_idx = stream_itr->second;
  }

  BubbleMap::mapped_type bubble;
  {
    auto locked_data_bubble_map = dataBubbleMap_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);
    if (bubble_itr == locked_data_bubble_map.get().end()) {
      return;
    }
    bubble = bubble_itr->second;
  }

  // grab the mutex from the stream map
  // auto guard = lockStream(stream_idx);
  bubble->unload();
}

void RCStreamInterface::resetBubble(const std::string &stream_name) {
  // get the corresponding stream.
  auto stream_idx = streamNames_->locked().get().at(stream_name);

  // extract the interval
  IntervalMap::mapped_type interval;
  {
    auto locked_stream_indices = streamIndices_.locked();
    auto interval_itr = locked_stream_indices.get().find(stream_idx);
    if (interval_itr == locked_stream_indices.get().end()) {
      return;
    }
    interval = interval_itr->second;
  }

  // remove the bubble
  auto bubble =
      dataBubbleMap_->locked()
          .get()
          .emplace(stream_idx, std::make_shared<robochunk::base::DataBubble>())
          .first->second;
  auto &stream = stream_map_->locked().get().at(stream_idx);
  auto guard = lockStream(stream_idx);
  bubble->initialize(stream.first);
  bubble->setIndices(interval.first, interval.second);
}
#endif

void RCStreamInterface::resetStream(const std::string &stream_name) {
  // get the corresponding stream.
  auto stream_idx = streamNames_->locked().get().at(stream_name);

  // remove the bubble and stream indices
  /// dataBubbleMap_->locked().get().erase(stream_idx);
  data_bubble_map_->locked().get().erase(stream_idx);
  streamIndices_.locked().get().erase(stream_idx);
}

void RCStreamInterface::write() {
  // Lock all the streams and set the saved flag
  std::deque<RWGuard> stream_locks;
  decltype(common::utils::getRefs<FieldMap>(
      streamNames_->locked().get())) stream_name_refs;
  {
    auto locked_names = streamNames_->locked();
    if (data_saved_)
      return;
    data_saved_ = true;
    stream_name_refs = common::utils::getRefs(locked_names.get());
    for (auto &stream_name : locked_names.get()) {
      stream_locks.emplace_back(lockStream(stream_name.second, false, true));
    }
  }

  // Write the streams out.
  for (auto &stream_itr : stream_name_refs) write(stream_itr.get().second);
}

void RCStreamInterface::write(const std::string &stream_name) {
  // Get the stream index from the stream name.
  auto stream_idx = streamNames_->locked().get().at(stream_name);
  write(stream_idx);
}

void RCStreamInterface::write(const uint32_t &stream_idx) {
  // Get the bubble from the stream index.
  DataBubbleMap::mapped_type data_bubble;
  {
    auto locked_data_bubble_map = data_bubble_map_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);

    // Exit if there is no bubble.
    if (bubble_itr == locked_data_bubble_map.get().end())
      return;

    data_bubble = bubble_itr->second;
  }

  // Get the serializer, exit if it doesn't exist.
  auto &data_stream = data_stream_map_->locked().get().at(stream_idx);
  auto writer = data_stream.second;
  if (writer == nullptr)
    return;

  // Serialize the bubble.
  bool bubble_has_msgs = false;
  Interval bubble_indices;
  {
    auto guard = lockStream(stream_idx, false, true);
    bubble_has_msgs = data_bubble->size() > 0;

    auto message_itr = data_bubble->begin();
    for (; message_itr != data_bubble->end(); ++message_itr) {
      // serialize the message.
      auto message = *message_itr;
      auto time_stamp = message->get_timestamp();
      auto write_status_index = writer->write(*message);
      // Set the bubble indices and time range.
      if (message_itr == data_bubble->begin()) {
        bubble_indices.first = write_status_index;
        timeRange_.first = time_stamp;
      }
      bubble_indices.second = write_status_index;
      timeRange_.second = time_stamp;
    }
  }

  // Add the indices to the indices map.
  if (bubble_has_msgs) {
    auto locked_stream_indices = streamIndices_.locked();
    auto interval_itr_bool =
        locked_stream_indices.get().emplace(stream_idx, bubble_indices);

    // If the bubble indices already existed, then update the end index.
    if (!interval_itr_bool.second)
      interval_itr_bool.first->second.second = bubble_indices.second;
  }
}

RCStreamInterface::RWGuard RCStreamInterface::lockStream(
    const FieldMap::mapped_type &stream_idx, bool read, bool write) {
  RWGuard stream_locks = [&]() {
    auto locked_stream_map = data_stream_map_->locked();
    auto rc_stream_itr = locked_stream_map.get().find(stream_idx);
    if (rc_stream_itr == locked_stream_map.get().end()) {
      throw std::runtime_error("could not load data for stream index " +
                               stream_idx);
    }
    RWGuard rwg;
    rwg.read = {rc_stream_itr->second.read_mtx, std::defer_lock};
    rwg.write = {rc_stream_itr->second.write_mtx, std::defer_lock};
    return rwg;
  }();

  if (read && write)
    std::lock(stream_locks.read, stream_locks.write);
  else if (read)
    stream_locks.read.lock();
  else if (write)
    stream_locks.write.lock();

  return stream_locks;
}

bool RCStreamInterface::insert(const std::string &stream_name,
                               storage::VTRMessage &vtr_msg) {
  FieldMap::mapped_type stream_idx;
  {
    // Get the stream index.
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(stream_name);
    if (stream_itr == locked_stream_names.get().end()) {
      LOG(WARNING) << "Stream " << stream_name << " not tied to this vertex!";
      return false;
    }
    stream_idx = stream_itr->second;
  }

  DataBubbleMap::mapped_type data_bubble;
  {
    auto locked_data_bubble_map = data_bubble_map_->locked();
    auto bubble_itr_bool = locked_data_bubble_map.get().emplace(
        stream_idx, std::make_shared<DataBubble>());
    data_bubble = bubble_itr_bool.first->second;

    // If insert was successful, we need to intialize the new bubble.
    if (bubble_itr_bool.second) {
      data_bubble->initialize(
          data_stream_map_->locked().get().at(stream_idx).first);
    }
  }

  data_bubble->insert(vtr_msg);
  return true;
}

RCStreamInterface::DataBubblePtr RCStreamInterface::getDataBubbleAndStreamIndex(
    const std::string &streamName, FieldMap::mapped_type &stream_idx) {
  // Get the stream index.
  {
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(streamName);
    if (stream_itr == locked_stream_names.get().end()) {
      // LOG(WARNING) << "Stream " << streamName << " not tied to this vertex!";
      return nullptr;
    }
    stream_idx = stream_itr->second;
  }

  // Get the data bubble.
  DataBubbleMap::mapped_type data_bubble;
  {
    auto locked_data_bubble_map = data_bubble_map_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);
    if (bubble_itr == locked_data_bubble_map.get().end())
      return nullptr;

    data_bubble = bubble_itr->second;
  }
  return data_bubble;
}

}  // namespace pose_graph
}  // namespace vtr
