#pragma once

#include <vtr_vision/types.hpp>

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

// PCL
#if 0
#include <pcl_ros/point_cloud.hpp>          //this file had not been ported to ROS2 yet
#include <pcl_ros/segmentation/sac_segmentation.hpp>
#endif

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

#if 0
/////////////////////////////////////////////////////////////////////////////////
/// @brief Estimates a plane from a PCL point cloud
/////////////////////////////////////////////////////////////////////////////////
bool estimatePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              const double distance_thresh,
                              pcl::ModelCoefficients &coefficients,
                              pcl::PointIndices &inliers);
#endif

/////////////////////////////////////////////////////////////////////////////////
/// @brief Estimat the distance from a plane
/////////////////////////////////////////////////////////////////////////////////
double estimatePlaneDepth( const Eigen::Vector3d &point,
                           const Eigen::Vector4f &coefficients);

}
}
