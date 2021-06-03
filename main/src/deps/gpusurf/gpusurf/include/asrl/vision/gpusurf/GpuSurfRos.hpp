#ifndef ASRL_ROS_CREATE_GPUSURF_HPP
#define ASRL_ROS_CREATE_GPUSURF_HPP

/**
 * @file   RosCreateGpuSurf.hpp
 * @author Paul Furgale <paul.furgale@utoronto.ca>
 * @date   Sat Nov 13 12:04:33 2010
 * 
 * @brief  Functions to create GPU-SURF detectors
 *         initialized from a ROS NodeHandle
 * 
 * 
 */

#include "GpuSurfStereoDetector.hpp"
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace asrl {

  typedef boost::shared_ptr<GpuSurfDetector> GpuSurfDetectorPtr;
  typedef boost::shared_ptr<const GpuSurfDetector> GpuSurfDetectorConstPtr;
  typedef boost::shared_ptr<GpuSurfStereoDetector> GpuSurfStereoDetectorPtr;
  typedef boost::shared_ptr<const GpuSurfStereoDetector> GpuSurfStereoDetectorConstPtr;

  /** 
   * Creates a GpuSurfDetector object by grabbing parameters from a ROS NodeHandle
   * 
   * @param parameters the nodehandle that contains the parameters to initialize the detector.
   * 
   * @return a boost::shared_ptr to an initialized detector object
   */
  GpuSurfDetectorPtr createGpuSurfDetector(const ros::NodeHandle & parameters);

  /** 
   * Creates a GpuSurfStereoDetector object by grabbing parameters from a ROS NodeHandle
   * 
   * @param parameters the nodehandle that contains the parameters to initialize the detector.
   * 
   * @return a boost::shared_ptr to an initialized detector object
   */
  GpuSurfStereoDetectorPtr createGpuSurfStereoDetector(const ros::NodeHandle & parameters);

} // namespace asrl


#endif /* ASRL_ROS_CREATE_GPUSURF_HPP */
