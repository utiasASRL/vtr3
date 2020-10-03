////////////////////////////////////////////////////////////////////////////////
/// @brief Header file for the ASRL vision package
/// @details Several convenient typedefs
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

// External
#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>

namespace vtr {
namespace vision {

// Feature keypoints
typedef cv::KeyPoint Keypoint;
typedef std::vector<Keypoint> Keypoints;
typedef Eigen::Map<Eigen::Matrix<float,2,Eigen::Dynamic>,
  Eigen::Unaligned, Eigen::Stride<sizeof(Keypoint),sizeof(float)> > KeypointsMap;
typedef Eigen::Map<const Eigen::Matrix<float,2,Eigen::Dynamic>,
Eigen::Unaligned, Eigen::Stride<sizeof(Keypoint)/sizeof(float),0> > KeypointsConstMap;

// Camera points
typedef Eigen::Vector2f CameraPoint;
typedef Eigen::Matrix<float,2,Eigen::Dynamic> CameraPoints;
typedef Eigen::Vector3f HomogeneousPoint;
typedef Eigen::Matrix<float,3,Eigen::Dynamic> HomogeneousPoints;
// To convert CameraPoints to HomogeneousPoints,
// homo = cam.colwise().homogeneous();
// To convert HomogeneousPoints to CameraPoints,
// cam = homo.hnormalized()


// Whitened measurement errors
typedef Eigen::Matrix<double,1,Eigen::Dynamic> ErrorList;
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MeasVarList;

} // namespace vision
} // namespace vtr_vision
