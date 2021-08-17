#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vtr_storage/data_stream_writer.hpp>

template <class MessageType>
class Recorder : public rclcpp::Node {
 public:
  /** \brief Constructor */
  Recorder(const std::string &data_dir, const std::string &stream_name,
           const std::string &topic)
      : Node("recorder"), writer_(data_dir, stream_name) {
    data_subscription_ = this->create_subscription<MessageType>(
        topic, 100,
        std::bind(&Recorder::msgCallback, this, std::placeholders::_1));
  }

 private:
  /** \brief Callback for XB3 image msgs */
  void msgCallback(std::shared_ptr<MessageType> msg) {
    auto stamp = msg->header.stamp;
    auto time_epoch = stamp.sec * 1e9 + stamp.nanosec;
    std::cout << "Received message at time: " << time_epoch << "\n";
    writer_.write(vtr::storage::VTRMessage(*msg));
  }

  /** \brief Subscriber for images */
  typename rclcpp::Subscription<MessageType>::SharedPtr data_subscription_;

  /** \brief Image writer */
  vtr::storage::DataStreamWriter<MessageType> writer_;
};