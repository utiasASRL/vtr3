#pragma once

#include <map>

#include "rcutils/types.h"

#include <vtr_logging/logging.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_storage/data_stream_reader.hpp>

namespace vtr {
namespace storage {

/**
 * \brief DataBubble class. Container class for a specified range of messages
 * in a robochunk stream. Allows for access and storage of messages in memory.
 */
class DataBubble {
 public:
  using Index = int32_t;
  using TimeStamp = rcutils_time_point_value_t;
  using VTRTimeStamp = vtr_messages::msg::TimeStamp;
  using VTRMessagePtr = std::shared_ptr<VTRMessage>;

  /** \brief Bubble indices into a robochunk stream. */
  struct ChunkIndices {
    /** \brief Start index into the stream. */
    Index start_index = 0;
    /** \brief Stop index into the stream. */
    Index stop_index = 0;
    /** \brief Start time into the stream. */
    TimeStamp start_time = 0;
    /** \brief Stop time into the stream. */
    TimeStamp stop_time = 0;
  };

  using IndexMap = std::map<Index, VTRMessagePtr>;
  using TimeMap = std::map<TimeStamp, VTRMessagePtr>;
  using DataVec = std::vector<VTRMessagePtr>;

  DataBubble() = default;
  DataBubble(const DataBubble&) = delete;
  DataBubble(DataBubble&&) = delete;

  virtual ~DataBubble() { reset(); }

  DataBubble& operator=(const DataBubble&) = delete;
  DataBubble& operator=(DataBubble&&) = delete;

  bool isInitialized() { return data_stream_ != nullptr; }
  /**
   * \brief Initializes a data bubble with a data stream.
   * \param data_stream A pointer to the associated data stream.
   */
  void initialize(std::shared_ptr<DataStreamReaderBase> data_stream) {
    data_stream_ = data_stream;
  }

  void reset() {
    unload();
    data_stream_.reset();
  }

  /**
   * \brief loads all of the messages associated with this bubble into memory.
   * \param[in] data_vec whether or not to append to data vector for iterating.
   */
  void load(bool to_data_vec = false);

  /**
   * \brief loads the specified message based on local index
   * \details This is a local index (i.e. If the bubble wraps 20 messages then
   * the local index is in the range (0-19))
   */
  void load(Index local_idx, bool to_data_vec = false);

  /** \brief loads a specific message based on a time tag into memory. */
  void loadTime(TimeStamp time, bool to_data_vec = false);
  void load(VTRTimeStamp time, bool to_data_vec = false) {
    return loadTime(toTimeStamp(time), to_data_vec);
  }

  /** \brief unloads all data associated with the vertex. */
  void unload();

  /** \brief Checks to see if a message is loaded based on index. */
  bool isLoaded(Index local_idx) {
    if (!global_index_set_) throw std::runtime_error{"Global index not set."};
    return index_map_.count(local_idx + indices_.start_index);
  };
  bool isLoaded(VTRTimeStamp time) { return isLoaded(toTimeStamp(time)); }

  /** \brief Checks to see if a message is loaded based on time. */
  bool isLoaded(TimeStamp time) { return time_map_.count(time); }

  /**
   * \brief Inserts a message into the data vector, into the index and time
   * map if time or index is set.
   */
  void insert(const VTRMessage& vtr_message);

  /** \brief Retrieves a reference to the message. */
  VTRMessage retrieve(Index local_idx);

  /** \brief Retrieves a reference to the message. */
  VTRMessage retrieveTime(TimeStamp time);
  VTRMessage retrieve(VTRTimeStamp time) {
    return retrieveTime(toTimeStamp(time));
  }

  /** \brief Sets the indicies for this bubble. */
  bool setIndices(Index index_begin, Index index_end = 0);

  /** \brief Sets the Time indices for this bubble. */
  bool setTimeIndices(TimeStamp time_begin, TimeStamp time_end);
  bool setIndices(VTRTimeStamp time_begin, VTRTimeStamp time_end) {
    return setTimeIndices(toTimeStamp(time_begin), toTimeStamp(time_end));
  }

  /**
   * \brief Gets the size of the bubble (number of messages in data vector)
   * \return the size of the bubble.
   */
  Index size() const { return data_vec_.size(); }

  /**
   * \brief provides an iterator to the begining of the insertion vector. Used
   * to store data.
   */
  DataVec::iterator begin() { return data_vec_.begin(); }

  /** \brief provides an iterator to the end of the bubble. */
  DataVec::iterator end() { return data_vec_.end(); }

 private:
  /**
   * \brief loads the specified messages into memory based on an index range.
   * \details These are global indices of messages in the database
   */
  void loadIndexRange_(Index global_idx0, Index global_idx1, bool to_data_vec);

  /**
   * \brief loads a range of messages based on time tags into memory.
   * \param time0 Begining time stamp of the message to be loaded.
   * \param time1 End time stamp of the message to be loaded.
   */
  void loadTimeRange_(TimeStamp time0, TimeStamp time1, bool to_data_vec);

  TimeStamp toTimeStamp(const VTRTimeStamp& time) {
    return static_cast<TimeStamp>(time.nanoseconds_since_epoch);
  }

  /** \brief A pointer to the Robochunk stream. */
  std::shared_ptr<DataStreamReaderBase> data_stream_;

  /** \brief whether index and time range are set */
  bool global_index_set_ = false;
  bool timestamp_set_ = false;
  /** \brief The indices associated with this bubble. */
  ChunkIndices indices_;
  /** \brief maps global indices to data. */
  IndexMap index_map_;
  /** \brief maps timestamps to data. */
  TimeMap time_map_;
  /** \brief A vector that holds unstored data */
  DataVec data_vec_;
};
}  // namespace storage
}  // namespace vtr
