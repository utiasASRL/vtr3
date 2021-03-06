#pragma once

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "vtr_storage_common.hpp"

namespace vtr {
namespace storage {

class DataStreamBase {
 public:
  DataStreamBase(const std::string &data_directory_string,
                 const std::string &stream_name = "");
  virtual ~DataStreamBase();

 protected:
  rosbag2_cpp::StorageOptions storage_options_;
  rosbag2_cpp::ConverterOptions converter_options_;

  rcpputils::fs::path base_directory_;
  rcpputils::fs::path data_directory_;
  std::string stream_name_;
  bool opened_;
};
}  // namespace storage
}  // namespace vtr
