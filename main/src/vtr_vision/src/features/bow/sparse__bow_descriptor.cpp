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
 * \file sparse__bow_descriptor.cpp
 * \brief Source file for the ASRL vision package
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <omp.h>
#include <algorithm>
#include <numeric>

#include <vtr_vision/features/bow/sparse_bow_descriptor.hpp>

namespace vtr {
namespace vision {

template class SparseBOWDescriptor<unsigned>;

}
}  // namespace vtr
