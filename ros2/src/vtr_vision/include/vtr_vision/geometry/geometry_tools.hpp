#pragma once

#include <vtr_vision/types.hpp>

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

// PCL
#include <pcl/point_types.h>              // todo
#include <pcl/segmentation/sac_segmentation.h>

namespace vtr {
namespace vision {

/////////////////////////////////////////////////////////////////////////////////
/// @brief Triangulates a point from a rig and keypoints
/////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d triangulateFromRig(const RigCalibration & rig_calibration,
                                   const std::vector<cv::Point2f> & keypoints,
                                   const FeatureInfos & kp_infos = FeatureInfos(),
                                   double * covariance = nullptr);


/////////////////////////////////////////////////////////////////////////////////
/// @brief Triangulates a point from a set of cameras and keypoints
/////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d triangulateFromCameras(const CameraIntrinsics &intrinsics,
                                       const Transforms &extrinsics,
                                       const std::vector<cv::Point2f> & keypoints,
                                       const FeatureInfos & kp_infos = FeatureInfos(),
                                       double * covariance = nullptr);

/////////////////////////////////////////////////////////////////////////////////
/// @brief Estimates a plane from a PCL point cloud
/////////////////////////////////////////////////////////////////////////////////
bool estimatePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              const double distance_thresh,
                              pcl::ModelCoefficients &coefficients,
                              pcl::PointIndices &inliers);

/////////////////////////////////////////////////////////////////////////////////
/// @brief Estimat the distance from a plane
/////////////////////////////////////////////////////////////////////////////////
double estimatePlaneDepth( const Eigen::Vector3d &point,
                           const Eigen::Vector4f &coefficients);

}
}
