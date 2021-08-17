/**
 * \file incremental_bow_trainer.cpp
 * \brief Source file for the ASRL vision package
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/features/bow/sliding_window_bow_descriptor.hpp>

namespace vtr {
namespace vision {

template class SlidingWindowBOWDescriptor<unsigned>;

}
}  // namespace vtr