#pragma once

#include <memory>

#include <vtr_pose_graph/robochunk/base/chunk_serializer.hpp>

namespace robochunk {
namespace base {
class DataBubble {
 public:
  DataBubble() = default;
  ~DataBubble() = default;

  /// @brief Initializes a data bubble with a data stream.
  /// @param A pointer to the associated data stream.
  void initialize(std::shared_ptr<robochunk::base::ChunkStream> data_stream) {
    std::cout << "Initializing a data bubble from data stream with "
              << data_stream->data_directory_ << " and "
              << data_stream->stream_name_
              << " TODO yuchen call the correct function." << std::endl;
  }

  /// @brief Inserts a message into the bubble.
  template <typename MessageType>
  void insert(MessageType& message) {
    std::cout << "Inserting a data with"
              << message.header.sensor_time_stamp.nanoseconds_since_epoch
              << " TODO yuchen call the correct function." << std::endl;
  }

  /// @brief unloads all data associated with the vertex.
  void load() {
    std::cout << "Loading all data associated with the vertex"
              << " TODO yuchen call the correct function." << std::endl;
  }


  /// @brief unloads all data associated with the vertex.
  void unload() {
    std::cout << "Unloading all data associated with the vertex"
              << " TODO yuchen call the correct function." << std::endl;
  }


 private:
};
}  // namespace base
}  // namespace robochunk
