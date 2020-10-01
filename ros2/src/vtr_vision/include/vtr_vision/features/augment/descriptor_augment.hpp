////////////////////////////////////////////////////////////////////////////////
/// Source file for the ASRL vision package
/// @details
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

// External
#include <opencv2/features2d/features2d.hpp>
#include <memory>
#include <list>
#include <Eigen/Core>
#include <stdexcept>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

typedef float DescType;

class AugmentorBase;
////////////////////////////////////////////////////////////////////
/// This class applies descriptor augmentors
////////////////////////////////////////////////////////////////////
class AugmentorEngine {
public:
  /// Augment the primary camera's descriptors with other info
  /// @param[in] channel The descriptors and info to be combined
  /// @return The augmented descriptor
  cv::Mat_<DescType> augment(const ChannelFeatures & channel);

  /// Add a new augmentor at the end of the queue
  void add(const std::shared_ptr<AugmentorBase> & a) {
    augmentors_.push_back(a);
  }

private:
  /// The list of augmentors that will be applied
  std::list<std::shared_ptr<AugmentorBase> > augmentors_;
};

////////////////////////////////////////////////////////////////////
/// This class provides an descriptor augmentor interface
////////////////////////////////////////////////////////////////////
class AugmentorBase {
protected:
  /// Get the extra dimensions to be added to the descriptors
  virtual unsigned getDim(const ChannelFeatures & channel) const = 0;

  /// Populate a set of descriptors with data
  virtual void augment(cv::Mat_<DescType> * augmented, unsigned column,
                       const ChannelFeatures & channel) const = 0;

  // The number of cameras in the channel
  static unsigned getNumCams(const ChannelFeatures & channel) {
    return channel.cameras.size();
  }

  // Interface through the engine
  friend class AugmentorEngine;
};

////////////////////////////////////////////////////////////////////
/// This class applies the raw descriptor
////////////////////////////////////////////////////////////////////
class AugmentorDescriptor : public AugmentorBase {
public:
  /// Constructor
  AugmentorDescriptor(float sigma_d = 0.55f)
    : sigma_d_(sigma_d) {
  }

private:
  /// Get the extra dimensions to be added to the descriptors
  unsigned getDim(const ChannelFeatures & channel) const {
    return channel.cameras[0].descriptors.cols;
  }

  /// Augment a set of descriptors with metadata
  void augment(cv::Mat_<DescType> * augmented, unsigned column,
               const ChannelFeatures & channel) const;

  /// The cut-off descriptor distance
  float sigma_d_;
  // Interface through the engine
  friend class AugmentorEngine;
};


////////////////////////////////////////////////////////////////////
/// This class applies a disparity descriptor
////////////////////////////////////////////////////////////////////
class AugmentorDisparity : public AugmentorBase {
public:
  /// Constructor
  AugmentorDisparity(const RigCalibration & calibration,
                     float sigma_d, float sigma_z);

private:
  /// Get the extra dimensions to be added to the descriptors
  unsigned getDim(const ChannelFeatures & channel) const;

  /// Augment a set of descriptors with metadata
  void augment(cv::Mat_<DescType> * augmented, unsigned column,
               const ChannelFeatures & channel) const;

  /// Calibration for the camera rig
  RigCalibration calibration_;
  /// Disparity uncertainty in pixels -- sqrt(2) keypoint_uncertainty
  float sigma_d_;
  /// Range uncertainty in same units as baseline
  float sigma_z_;
  // Interface through the engine
  friend class AugmentorEngine;
};

////////////////////////////////////////////////////////////////////
/// This class applies a size descriptor
////////////////////////////////////////////////////////////////////
class AugmentorSize : public AugmentorBase {
public:
  /// Constructor
  AugmentorSize(float sigma_sz_log_base = 2.f)
    : sigma_sz_log_base_(sigma_sz_log_base) {
  }

private:
  /// Get the extra dimensions to be added to the descriptors
  unsigned int getDim(const ChannelFeatures & channel) const {
    (void)&channel;
    return 1;
  }

  /// Augment a set of descriptors with metadata
  void augment(cv::Mat_<DescType> * augmented, unsigned offset,
               const ChannelFeatures & channel) const;

  /// Expected size ratio between matching keypoints
  float sigma_sz_log_base_;
  // Interface through the engine
  friend class AugmentorEngine;
};

////////////////////////////////////////////////////////////////////
/// This class applies a pixel position descriptor
////////////////////////////////////////////////////////////////////
class AugmentorPixelPos : public AugmentorBase {
public:
  static const unsigned int D = 2;

  /// Constructor
  AugmentorPixelPos(float sigma_pixel_d = 200.f)
    : sigma_pixel_d_(sigma_pixel_d) {
  }

private:
  /// Get the extra dimensions to be added to the descriptors
  unsigned int getDim() const { return D; }

  /// Augment a set of descriptors with metadata
  void augment(cv::Mat_<DescType> * augmented,
               unsigned int offset,
               const ChannelFeatures & channel) const;

  /// Expected size ratio between matching keypoints
  float sigma_pixel_d_;
  // Interface through the engine
  friend class AugmentorEngine;
};

} // vision
} // asrl
