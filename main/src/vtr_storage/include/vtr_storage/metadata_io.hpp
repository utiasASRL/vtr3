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
 * \file metadata_io.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <string>

#include "vtr_storage/storage/bag_metadata.hpp"

namespace vtr {
namespace storage {

/// \note the following code is adapted from rosbag2 foxy

// Copyright 2018, Bosch Software Innovations GmbH.
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

// clang-format off
class MetadataIo
{
public:
  static constexpr const char * const metadata_filename = "metadata.yaml";

  virtual ~MetadataIo() = default;

  virtual void write_metadata(const std::string & uri, const BagMetadata & metadata);

  virtual BagMetadata read_metadata(const std::string & uri);

  virtual bool metadata_file_exists(const std::string & uri);

private:
  std::string get_metadata_file_name(const std::string & uri);
};

// clang-format on

}  // namespace storage
}  // namespace vtr
