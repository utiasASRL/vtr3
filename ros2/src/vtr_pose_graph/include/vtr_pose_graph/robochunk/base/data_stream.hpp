#pragma once

#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

namespace robochunk {
namespace base {
class ChunkSerializer {
 public:
  ChunkSerializer(fs::path data_directory, fs::path stream_name)
      : data_directory_{data_directory}, stream_name_{stream_name} {
    // std::cout << "TODO: yuchen call the correct function!" << std::endl;
  }
  ~ChunkSerializer() = default;

  fs::path data_directory_;
  fs::path stream_name_;
};
}  // namespace base
}  // namespace robochunk
