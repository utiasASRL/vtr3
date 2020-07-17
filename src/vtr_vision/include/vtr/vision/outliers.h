////////////////////////////////////////////////////////////////////////////////
/// @brief Convenience header for outlier rejection (RANSAC)
/// @details
///////////////////////////////////////////////////////////////////////////////

#pragma once

// RANSAC Methods
#include "vtr/vision/outliers/ransac/vanilla_ransac.h"

// Samplers
#include "vtr/vision/outliers/sampler/basic_sampler.h"
#include "vtr/vision/outliers/sampler/progressive_sampler.h"

// Sample Verifiers
#include "vtr/vision/outliers/sampler/verify_sample_indices.h"
#include "vtr/vision/outliers/sampler/verify_sample_subset_mask.h"
