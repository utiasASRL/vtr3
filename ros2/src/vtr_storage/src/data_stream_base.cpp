// description
#include "vtr_storage/data_stream_base.hpp"

namespace vtr {
namespace storage {

DataStreamBase::DataStreamBase(const std::string &data_directory_string,
                               const std::string &stream_name)
    : base_directory_(rcpputils::fs::path(data_directory_string)),
      data_directory_(rcpputils::fs::path(data_directory_string)),
      stream_name_(stream_name),
      opened_(false) {
  if (stream_name != "") {
    data_directory_ /= stream_name;
  }
  storage_options_.uri = data_directory_.string();
  storage_options_.storage_id = "sqlite3";
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 0;

  converter_options_.input_serialization_format = "cdr";
  converter_options_.output_serialization_format = "cdr";
}

DataStreamBase::~DataStreamBase() {}
}  // namespace storage
}  // namespace vtr