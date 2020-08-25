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
      stream_map_(LockableStreamMapPtr()),
      streamIndices_(IntervalMap()) {
  dataBubbleMap_.reset(new LockableBubbleMap());
}

#if 0
RCStreamInterface::RCStreamInterface(
    const asrl::graph_msgs::Interval &timeRange,
    const LockableFieldMapPtr &stream_names,
    const LockableStreamMapPtr &streamMap,
    const google::protobuf::RepeatedPtrField<asrl::graph_msgs::IntervalIndex>
        &streamIndices)
    : timeRange_(Interval(timeRange.idx1(), timeRange.idx2())),
      streamNames_(stream_names),
      stream_map_(streamMap),
      streamIndices_(IntervalMap()),
      data_saved_(false) {
  keyFrameTime_.set_nanoseconds_since_epoch(timeRange.idx2());
  dataBubbleMap_.reset(new LockableBubbleMap());
  for (auto it = streamIndices.begin(); it != streamIndices.end(); ++it) {
    streamIndices_.locked().get().emplace(it->nameidx(),
                                          Interval(it->idx1(), it->idx2()));

    // add to the data bubble
    auto &robochunk_io = stream_map_->locked().get().at(it->nameidx());
    if (robochunk_io.first != nullptr) {
      auto bubble =
          dataBubbleMap_->locked()
              .get()
              .emplace(it->nameidx(),
                       std::make_shared<robochunk::base::DataBubble>())
              .first->second;
      bubble->initialize(robochunk_io.first);
      bubble->setIndices(it->idx1(), it->idx2());
    }
  }
}

void RCStreamInterface::serializeStreams(
    asrl::graph_msgs::Interval *timeRange,
    google::protobuf::RepeatedPtrField<asrl::graph_msgs::IntervalIndex>
        *streamIndices) const {
  streamIndices->Clear();
  streamIndices->Reserve(streamIndices_.locked().get().size());
  for (auto &&it : common::utils::getRefs(streamIndices_.locked().get())) {
    asrl::graph_msgs::IntervalIndex *tmpMsg = streamIndices->Add();
    tmpMsg->set_nameidx(it.get().first);
    tmpMsg->set_idx1(it.get().second.first);
    tmpMsg->set_idx2(it.get().second.second);
  }

  timeRange->Clear();
  timeRange->set_idx1(timeRange_.first);
  timeRange->set_idx2(timeRange_.second);
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

void RCStreamInterface::resetStream(const std::string &stream_name) {
  // get the corresponding stream.
  auto stream_idx = streamNames_->locked().get().at(stream_name);

  // remove the bubble and stream indices
  dataBubbleMap_->locked().get().erase(stream_idx);
  streamIndices_.locked().get().erase(stream_idx);
}

void RCStreamInterface::addStreamIndices(const std::string &stream_name,
                                         const Interval &interval,
                                         bool overwrite) {
  if (streamNames_ == nullptr) {
    streamNames_.reset(new LockableFieldMap());
  }

  // add the stream names to the map if id does not exsist.
  FieldMap::mapped_type idx;
  {
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr_bool = locked_stream_names.get().emplace(
        stream_name, locked_stream_names.get().size());
    idx = stream_itr_bool.first->second;
  }

  // insert the index into the map.
  if (overwrite) {
    streamIndices_.locked().get()[idx] = interval;
  } else {
    streamIndices_.locked().get().emplace(idx, interval);
  }

  if (stream_map_ == nullptr) {
    LOG(ERROR) << "Streams have not been initialized for this run!!";
    return;
  }

  // Insert will return an existing BubblePtr or create a new one.  Initialize
  // is safe to run on an existing bubble. Retrieve bubble
  auto bubble =
      dataBubbleMap_->locked()
          .get()
          .emplace(idx, std::make_shared<robochunk::base::DataBubble>())
          .first->second;
  auto guard = lockStream(idx);
  bubble->initialize(stream_map_->locked().get().at(idx).first);
  bubble->unload();
  bubble->setIndices(interval.first, interval.second);
}

void RCStreamInterface::load(const std::string &stream_name) {
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
  auto guard = lockStream(stream_idx, true, false);
  bubble->load();
}

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

////////////////////////////////////////////////////////////////////////////
// RCStreamInterface::unload
////////////////////////////////////////////////////////////////////////////
void RCStreamInterface::load() {
  for (auto &&itr : common::utils::getRefs(dataBubbleMap_->locked().get())) {
    itr.get().second->load();
  }
}

////////////////////////////////////////////////////////////////////////////
// RCStreamInterface::unload
////////////////////////////////////////////////////////////////////////////
void RCStreamInterface::unload() {
  for (auto &&itr : common::utils::getRefs(dataBubbleMap_->locked().get())) {
    itr.get().second->unload();
  }
}

void RCStreamInterface::write(const std::string &stream_name) {
  // Get the stream index from the stream name.
  auto stream_idx = streamNames_->locked().get().at(stream_name);
  write(stream_idx);
}
void RCStreamInterface::write(const uint32_t &stream_idx) {
  // Get the bubble from the stream index.
  BubbleMap::mapped_type bubble;
  {
    auto locked_data_bubble_map = dataBubbleMap_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);

    // Exit if there is no bubble.
    if (bubble_itr == locked_data_bubble_map.get().end()) {
      return;
    }

    bubble = bubble_itr->second;
  }

  // Get the serializer, exit if it doesn't exist.
  auto &stream = stream_map_->locked().get().at(stream_idx);
  auto serializer = stream.second;
  if (serializer == nullptr) {
    return;
  }

  // Serialize the bubble.
  bool bubble_has_msgs = false;
  Interval bubble_indices;
  {
    auto guard = lockStream(stream_idx, false, true);
    bubble_has_msgs = bubble->size() > 0;
    auto message_itr = bubble->begin();
    for (; message_itr != bubble->end(); ++message_itr) {
      // serialize the message.
      auto &message = message_itr->second.baseMessage();
      auto &stamp = message.header().sensor_time_stamp();
      auto write_status = serializer->serialize(message);

      // Set the bubble indices and time range.
      if (message_itr == bubble->begin()) {
        bubble_indices.first = write_status.index;
        timeRange_.first = stamp.nanoseconds_since_epoch();
      }
      bubble_indices.second = write_status.index;
      timeRange_.second = stamp.nanoseconds_since_epoch();
    }
  }

  // Add the indices to the indices map.
  if (bubble_has_msgs) {
    auto locked_stream_indices = streamIndices_.locked();
    auto interval_itr_bool =
        locked_stream_indices.get().emplace(stream_idx, bubble_indices);

    // If the bubble indices already existed, then update the end index.
    if (!interval_itr_bool.second) {
      interval_itr_bool.first->second.second = bubble_indices.second;
    }
  }
}

void RCStreamInterface::write() {
  // Lock all the streams and set the saved flag
  std::deque<RWGuard> stream_locks;
  decltype(common::utils::getRefs<FieldMap>(
      streamNames_->locked().get())) stream_name_refs;
  {
    auto locked_names = streamNames_->locked();
    if (data_saved_) return;
    data_saved_ = true;
    stream_name_refs = common::utils::getRefs(locked_names.get());
    for (auto &stream_name : locked_names.get()) {
      stream_locks.emplace_back(lockStream(stream_name.second, false, true));
    }
  }

  // Write the streams out.
  for (auto &stream_itr : stream_name_refs) {
    write(stream_itr.get().second);
  }
}

bool RCStreamInterface::insert(const std::string &stream_name,
                               robochunk::msgs::RobochunkMessage msg) {
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

  // Get the data bubble.
  BubbleMap::mapped_type bubble;
  {
    auto locked_data_bubble_map = dataBubbleMap_->locked();
    auto bubble_itr_bool = locked_data_bubble_map.get().emplace(
        stream_idx, std::make_shared<robochunk::base::DataBubble>());
    bubble = bubble_itr_bool.first->second;

    // If insert was successful, we need to intialize the new bubble.
    if (bubble_itr_bool.second) {
      bubble->initialize(stream_map_->locked().get().at(stream_idx).first);
    }
  }

  // grab the mutex from the stream map
  // auto guard = lockStream(stream_idx);

  // insert the data
  bubble->insert(msg);
  return true;
}

RCStreamInterface::RWGuard RCStreamInterface::lockStream(
    const FieldMap::mapped_type &stream_idx, bool read, bool write) {
  RWGuard stream_locks = [&]() {
    auto locked_stream_map = stream_map_->locked();
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

#ifdef DEBUG  // Enable timing of collisions if debugging
  common::timing::SimpleTimer timer;
#endif
  if (read && write) {
    std::lock(stream_locks.read, stream_locks.write);
  } else if (read) {
    stream_locks.read.lock();
  } else if (write) {
    stream_locks.write.lock();
  }
#ifdef DEBUG
  if (timer.elapsedMs() > 10) {
    std::string stream_name = "";
    for (auto &&it : common::utils::getRefs(streamNames_->locked().get())) {
      if (it.get().second == stream_idx) {
        stream_name = it.get().first;
      }
    }
    LOG(WARNING) << "Waited for " << stream_name << " " << timer;
  }
#endif
  return stream_locks;
}
#endif
}  // namespace pose_graph
}  // namespace vtr
