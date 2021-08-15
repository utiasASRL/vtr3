/**
 * \file sparse_bow_descriptor_bow_trainer.cpp
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
