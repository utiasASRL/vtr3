////////////////////////////////////////////////////////////////////////////////
/// @brief BasicSampler.cpp Source file for the ASRL vision package
/// @details This file instantiates the VanillaRansac class, which provides
///          a basic RANSAC algorithm
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

// ASRL
#include "vtr/vision/outliers/ransac/vanilla_ransac.h"
#include "vtr/vision/sensors/sensor_model_base.h"


namespace vtr {
namespace vision {

template class RansacBase<Eigen::Matrix3d>;
template class RansacBase<Eigen::Matrix4d>;

template class VanillaRansac<Eigen::Matrix3d>;
template class VanillaRansac<Eigen::Matrix4d>;

} // namespace vision
} // namespace vtr