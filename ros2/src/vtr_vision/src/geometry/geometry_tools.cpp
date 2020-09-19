#include <vtr_vision/geometry/geometry_tools.hpp>
#include <vtr_logging/logging.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace vtr {
namespace vision {

/////////////////////////////////////////////////////////////////////////////////
// @brief Triangulates a point from a rig and keypoints
/////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d triangulateFromRig(const RigCalibration & rig_calibration,
                                   const std::vector<cv::Point2f> & keypoints,
                                   const FeatureInfos & kp_infos,
                                   double * covariance) {
    return triangulateFromCameras(rig_calibration.intrinsics, rig_calibration.extrinsics,
                                  keypoints, kp_infos, covariance);
}

/////////////////////////////////////////////////////////////////////////////////
/// @brief Triangulates a point (linearly) from a set of cameras and keypoints
/////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d triangulateFromCameras(const CameraIntrinsics & intrinsics,
                                       const Transforms & extrinsics,
                                       const std::vector<cv::Point2f> & keypoints,
                                       const vision::FeatureInfos & kp_infos,
                                       double * covariance) {
    // sanity check
    assert(intrinsics.size() == keypoints.size());
    assert(extrinsics.size() == keypoints.size());

    // make up the solution matrix
    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(keypoints.size()*2,4);
    for(unsigned ii = 0; ii < keypoints.size(); ii++) {
        // grab each observation and add it to the matrix to solve
        Eigen::Matrix<double,3,4> P = intrinsics[ii]*extrinsics[ii].matrix().block(0,0,3,4);
        Z.row(ii*2) = keypoints[ii].x*P.row(2) - P.row(0);
        Z.row(ii*2+1) = keypoints[ii].y*P.row(2) - P.row(1);
    }

    // solve the linear triangulation problem using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd V = svd.matrixV();

    // extract the 3D point
    Eigen::Vector3d X = V.col(3).hnormalized();

    // calculate the covariance if required
    if (covariance) {
      // wrap the covariance and zero-initialize
      Eigen::Map<Eigen::Matrix3d> cov_map(covariance);
      cov_map.setZero();

      // loop over the contribution of each measurement
      for(unsigned ii = 0; ii < keypoints.size(); ii++) {
        // Reproject the solved point for linearization
        Eigen::Matrix<double,3,4> P = intrinsics[ii]*extrinsics[ii].matrix().block(0,0,3,4);
        Eigen::Vector3d xii = P*X.homogeneous();
        const auto & xii2 = xii(2); auto xii2_2 = xii2*xii2; // helpers

        // homogeneous to cartesian Jacobian for image points
        Eigen::Matrix<double,2,3> h2c_jac;
        h2c_jac << 1/xii2, 0, -xii(0)/xii2_2,
            0, 1/xii2, -xii(1)/xii2_2;

        // full camera projection Jacobian
        auto jac = h2c_jac * P.leftCols<3>();
        // sum the (linearized) precision contributions of each measurement
        cov_map += jac.transpose()*kp_infos[ii].covariance.inverse()*jac;
      }
      // from precision matrix to covariance matrix
      cov_map = cov_map.inverse().eval();
    }

    // return the linearly triangulated 3D point
    return X;
}
#if 0
/////////////////////////////////////////////////////////////////////////////////
/// @brief Estimates a plane from a PCL point cloud
/////////////////////////////////////////////////////////////////////////////////
bool estimatePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              const double distance_thresh,
                              pcl::ModelCoefficients &coefficients,
                              pcl::PointIndices &inliers) {


  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_thresh);
  seg.setInputCloud (cloud);
  seg.segment (inliers, coefficients);

  return !inliers.indices.empty();
}
#endif
/////////////////////////////////////////////////////////////////////////////////
/// @brief Estimate the distance from a plane
/////////////////////////////////////////////////////////////////////////////////
double estimatePlaneDepth(const Eigen::Vector3d &point,
                           const Eigen::Vector4f &coefficients) {

  // rename for clarity
  const double &a = point(0);
  const double &b = point(1);
  const double &c = point(2);
  const float &pa = coefficients(0);
  const float &pb = coefficients(1);
  const float &pc = coefficients(2);
  const float &pd = coefficients(3);

  // numerator
  double num = std::fabs(a*pa+b*pb+c*pc+pd);

  // denominator
  double den = std::sqrt(pa*pa+pb*pb+pc*pc);

  //result
  double dist = num/den;

  return dist;
}

}}
