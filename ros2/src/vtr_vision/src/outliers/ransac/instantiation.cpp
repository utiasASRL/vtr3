////////////////////////////////////////////////////////////////////////////////
/// @brief instantiation.cpp Source file for the VTR vision package
/// @details This file instantiates the VanillaRansac class, which provides
///          a basic RANSAC algorithm
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#include "vtr_vision/outliers/ransac/vanilla_ransac.hpp"
#include "vtr_vision/sensors/sensor_model_base.hpp"


namespace vtr {
namespace vision {

template class RansacBase<Eigen::Matrix3d>;
template class RansacBase<Eigen::Matrix4d>;

template class VanillaRansac<Eigen::Matrix3d>;
template class VanillaRansac<Eigen::Matrix4d>;

} // namespace vision
} // namespace vtr_vision