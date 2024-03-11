#include <asrl/vision/gpusurf/GpuSurfRos.hpp>
//#include <asrl/ros/rosparam.hpp>

namespace asrl {
  
  // \todo When Andrew is done with his paper. Re-link this to libasrl
  namespace rosutil {
    template<typename T>
    inline void param(const ros::NodeHandle & nh, 
		      const std::string & key, 
		      T & param, 
		      const T & defaultValue)
    {
      if(!nh.getParam(key,param))
	{
	  nh.setParam(key,defaultValue);
	  param = defaultValue;
	}      
    }

  } // rosutil

  // Helper functions.
  void init(GpuSurfConfiguration & config, const ros::NodeHandle & parameters)
  {
    // ROS has no param<float> defined. We must use a temp double.
    double tmp;

    // threshold(0.1f),
    asrl::rosutil::param<double>(parameters, "threshold",tmp,0.001);
    config.threshold = tmp;

    // nOctaves(4),
    asrl::rosutil::param<int>(parameters, "nOctaves",config.nOctaves,4);

    // nIntervals(4),
    asrl::rosutil::param<int>(parameters, "nIntervals",config.nIntervals,4);

    // initialScale(2.f),
    asrl::rosutil::param<double>(parameters, "initialScale",tmp,2.0);
    config.initialScale = tmp;

    // l1(3.f/1.5f),
    asrl::rosutil::param<double>(parameters, "l1",tmp,3.0/1.5);
    config.l1 = tmp;
    
    // l2(5.f/1.5f),
    asrl::rosutil::param<double>(parameters, "l2",tmp,5.0/1.5);
    config.l2 = tmp;
        
    // l3(3.f/1.5f),
    asrl::rosutil::param<double>(parameters, "l3",tmp,3.0/1.5);
    config.l3 = tmp;
        
    // l4(1.f/1.5f),
    asrl::rosutil::param<double>(parameters, "l4",tmp,1.0/1.5);
    config.l4 = tmp;
        
    // edgeScale(0.81f),
    asrl::rosutil::param<double>(parameters, "edgeScale",tmp,0.81);
    config.edgeScale = tmp;
    
    // initialStep(1),
    asrl::rosutil::param<int>(parameters, "initialStep",config.initialStep,1);

    // targetFeatures(1000),
    asrl::rosutil::param<int>(parameters, "targetFeatures",config.targetFeatures,1000);

    // detector_threads_x(16),
    asrl::rosutil::param<int>(parameters, "detector_threads_x",config.detector_threads_x,16);

    // detector_threads_y(4),
    asrl::rosutil::param<int>(parameters, "detector_threads_y",config.detector_threads_y,4);

    // nonmax_threads_x(16),
    asrl::rosutil::param<int>(parameters, "nonmax_threads_x",config.nonmax_threads_x,16);

    // nonmax_threads_y(16),
    asrl::rosutil::param<int>(parameters, "nonmax_threads_y",config.nonmax_threads_y,16);

    // regions_horizontal(1),
    asrl::rosutil::param<int>(parameters, "regions_horizontal",config.regions_horizontal,2);

    // regions_vertical(1),
    asrl::rosutil::param<int>(parameters, "regions_vertical",config.regions_vertical,2);

    // regions_target(8192)
    asrl::rosutil::param<int>(parameters, "regions_target",config.regions_target,1000);

  }

  void initStereo(GpuSurfStereoConfiguration & config, const ros::NodeHandle & parameters)
  {
    // ROS has no param<float> defined. We must use a temp double.
    double tmp;
    
    // Initialize the base SURF parameters
    init(config,parameters);

    // Now the special stereo parameters.
    
    // stereoDisparityMinimum(0.f),
    asrl::rosutil::param<double>(parameters, "stereoDisparityMinimum",tmp,0.0);
    config.stereoDisparityMinimum = tmp;

    // stereoDisparityMaximum(120.f),
    asrl::rosutil::param<double>(parameters, "stereoDisparityMaximum",tmp,120.0);
    config.stereoDisparityMaximum = tmp;

    // stereoCorrelationThreshold(0.9f),
    asrl::rosutil::param<double>(parameters, "stereoCorrelationThreshold",tmp,0.9);
    config.stereoCorrelationThreshold = tmp;

    // stereoYTolerance(1.f),
    asrl::rosutil::param<double>(parameters, "stereoYTolerance",tmp,1.0);
    config.stereoYTolerance = tmp;

    // stereoScaleTolerance(0.8f)
    asrl::rosutil::param<double>(parameters, "stereoScaleTolerance",tmp,0.8);
    config.stereoScaleTolerance = tmp;
  }

  GpuSurfDetectorPtr createGpuSurfDetector(const ros::NodeHandle & parameters)
  {
    GpuSurfConfiguration config;
    init(config,parameters);
    return GpuSurfDetectorPtr(new GpuSurfDetector(config));
  }

  GpuSurfStereoDetectorPtr createGpuSurfStereoDetector(const ros::NodeHandle & parameters)
  {
    GpuSurfStereoConfiguration config;
    initStereo(config,parameters);
    
    return GpuSurfStereoDetectorPtr(new GpuSurfStereoDetector(config));
  }

} // namespace asrl
