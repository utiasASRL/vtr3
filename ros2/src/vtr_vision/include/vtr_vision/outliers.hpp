////////////////////////////////////////////////////////////////////////////////
/// @brief Convenience header for outlier rejection (RANSAC)
/// @details
///////////////////////////////////////////////////////////////////////////////

#pragma once

// RANSAC Methods
#include "vtr_vision/outliers/ransac/vanilla_ransac.hpp"

// Samplers
#include "vtr_vision/outliers/sampler/basic_sampler.hpp"
#include "vtr_vision/outliers/sampler/progressive_sampler.hpp"

// Sample Verifiers
#include "vtr_vision/outliers/sampler/verify_sample_indices.hpp"
#include "vtr_vision/outliers/sampler/verify_sample_subset_mask.hpp"
