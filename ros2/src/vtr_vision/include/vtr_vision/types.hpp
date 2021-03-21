////////////////////////////////////////////////////////////////////////////////
/// @brief Header defining types used in VTR vision package
/// @details
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <lgmath.hpp>
#include <vtr_common/utils/hash.hpp>
#include <vtr_logging/logging.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

// opencv definitions
#include <opencv2/core/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/version.hpp>  // defines CV_MAJOR_VERSION
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>  // defines HAVE_OPENCV_CUDAFEATURES2D

#include <array>
#include <memory>
#include <vector>

namespace vtr {
namespace vision {

// this is important for OpenCV and SURF GPU multithreaded operations
extern std::mutex __gpu_mutex__;

////////////////////////////////////////////////////////////////////////////////
// Image storage, the analog of robochunk Images.proto

/// An image from a single channel type and camera
struct Image {
  /** \brief  default constructor
   */
  Image() = default;

  /** \brief copy constructor
   *
   * @param img
   */
  Image(const Image &img) {
    stamp = img.stamp;
    name = img.name;
    data = img.data.clone();
  }

  /** \brief copy assignment (required due to defaulted deletion in the
   * compiler)
   */
  Image &operator=(const Image &img) {
    stamp = img.stamp;
    name = img.name;
    data = img.data.clone();
    return *this;
  }

  /** \brief move constructor
   */
  Image(Image &&img) noexcept {
    stamp = img.stamp;
    name = img.name;
    data = img.data;
  }
  /// The image timestamp (ns epoch time)
  uint64_t stamp;
  /// The name of the camera (e.g. left, right)
  std::string name;
  /// The OpenCV image
  cv::Mat data;
};

/// Images from a camera rig for a single channel type
struct ChannelImages {
  /** \brief default constructor
   */
  ChannelImages() = default;

  /** \brief copy constructor
   */
  ChannelImages(const ChannelImages &channel) {
    name = channel.name;
    cameras = channel.cameras;
  }

  /** \brief copy assignment (required due to defaulted deletion in the
   * compiler)
   */
  ChannelImages &operator=(const ChannelImages &channel) {
    name = channel.name;
    cameras = channel.cameras;
    return *this;
  }

  /** \brief move constructor
   */
  ChannelImages(ChannelImages &&channel) noexcept {
    name = channel.name;
    for (auto &camera : channel.cameras) {
      cameras.emplace_back(std::move(camera));
    }
  }
  /// The name of the channel (e.g. grey, dessert, forest)
  std::string name;
  /// The images from the different cameras in the rig
  std::vector<Image> cameras;
};

/// Images from a camera rig
struct RigImages {
  /** \brief default constructor */
  RigImages() = default;

  /** \brief copy constructor */
  RigImages(const RigImages &channel) {
    name = channel.name;
    channels = channel.channels;
  }

  /**
   * \brief copy assignment (required due to defaulted deletion in the compiler)
   */
  RigImages &operator=(const RigImages &channel) {
    name = channel.name;
    channels = channel.channels;
    return *this;
  }

  /** \brief move constructor */
  RigImages(RigImages &&rig) noexcept {
    name = rig.name;
    for (auto &channel : rig.channels) {
      channels.emplace_back(std::move(channel));
    }
  }
  ~RigImages() = default;
  /// The name of the rig (e.g. front-xb3, rear-visensor)
  std::string name;
  /// The images from all the different channel types
  std::vector<ChannelImages> channels;
};

////////////////////////////////////////////////////////////////////////////////
// Simple matches for interacting with Steam

using SimpleMatch = std::pair<unsigned, unsigned>;
using SimpleMatches = std::vector<SimpleMatch>;

struct ChannelMatches {
  std::string name;
  SimpleMatches matches;
};

struct RigMatches {
  std::string name;
  std::vector<ChannelMatches> channels;
};

typedef std::vector<RigMatches> SuiteMatches;

////////////////////////////////////////////////////////////////////////////////
// Keypoints

/// Use OpenCV's keypoint, since we need to operator on this type
typedef cv::KeyPoint Keypoint;
typedef std::vector<Keypoint> Keypoints;

////////////////////////////////////////////////////////////////////////////////
// Descriptors

enum struct FeatureImpl { UNKNOWN = 0, OPENCV_ORB, ASRL_GPU_SURF };
struct FeatureType {
  /// The implementation used for feature extraction
  FeatureImpl impl;
  /// The dimension of the descriptor
  unsigned dims;
  /// The number of bytes in the descriptor
  unsigned bytes_per_desc;
  /// Whether the descriptor is upright or rotated (rotation invariant)
  bool upright;
};

////////////////////////////////////////////////////////////////////////////////
// Collections of features, mirroring Features.proto but with usable types

/// Extra keypoint appearance info not covered by OpenCV's definition.
/// The split here differs from the protobuf split between keypoint and info.
struct FeatureInfo {
  /// Detector Laplacian bit
  bool laplacian_bit;
  /// Keypoint precision (1/sigma^2), where sigma is in pixels
  float precision;

  Eigen::Matrix2d covariance;
  FeatureInfo(bool lb, float pr) : laplacian_bit(lb), precision(pr) {
  }
  FeatureInfo() : laplacian_bit(false), precision(0.0f) {
  }
};
typedef std::vector<FeatureInfo> FeatureInfos;

/// The collection of features from an image for a single channel type
struct Features {
  /// The camera name (e.g. left, right)
  std::string name;

  /// The keypoints detected in the image
  Keypoints keypoints;

  /// The extra keypoint information not included in the OpenCV keypoint
  FeatureInfos feat_infos;

  /// The descriptors stored as a binary blob
  cv::Mat descriptors;

  /// The descriptors stored as a binary blob
#if CV_MAJOR_VERSION >= 3 && defined(HAVE_OPENCV_CUDAFEATURES2D)
  cv::cuda::GpuMat gpu_descriptors;
#endif

  /// The type of feature extracted
  FeatureType feat_type;
};

/// The collection of features from each camera in a rig for one channel type
struct ChannelFeatures {
  /// The channel name (e.g. grey, dessert, forest)
  std::string name;
  /// The features for each camera in the rig
  std::vector<Features> cameras;
  // Matches rig_matches;
  /// Whether the features are fully matched.
  /// This means every feature has a match, and the indices are match aligned.
  bool fully_matched;
};

/// The collection of features for every camera and channel in a camera rig
struct RigFeatures {
  /// The rig name (e.g. front-xb3, rear-visensor)
  std::string name;
  /// The features for each channel in the rig
  std::vector<ChannelFeatures> channels;
};

typedef std::vector<RigFeatures> SuiteFeatures;

////////////////////////////////////////////////////////////////////////////////
// Landmark Matches
struct PersistentId {
  PersistentId() : stamp(-1), robot(-1) {
  }
  PersistentId(uint64_t s, uint32_t r) : stamp(s), robot(r) {
  }
  uint64_t stamp;
  uint32_t robot;
};

struct LandmarkId {
  LandmarkId() : index(-1), channel(-1), camera(-1), rig(-1), persistent() {
  }

  LandmarkId(const PersistentId &p, uint32_t ri, uint32_t ch)
      : index(-1), channel(ch), camera(-1), rig(ri), persistent(p) {
  }

  // THIS ORDER IS DIFFERENT THAN NORMAL, READ CAREFULLY
  LandmarkId(const PersistentId &p, uint32_t ri, uint32_t ch, uint32_t idx,
             uint32_t cm)
      : index(idx), channel(ch), camera(cm), rig(ri), persistent(p) {
  }
  uint32_t index;
  uint32_t channel;
  uint32_t camera;
  uint32_t rig;
  PersistentId persistent;
};
typedef std::vector<LandmarkId> LandmarkIds;

inline std::ostream &operator<<(std::ostream &os, const LandmarkId &id) {
  os << "ro" << id.persistent.robot << ",t" << id.persistent.stamp << ",ri"
     << id.rig << ",ch" << id.channel << ",i" << id.index << ",ca" << id.camera;
  return os;
}

// Equality comparison so it can be used in a sorted map
inline bool operator==(const vtr::vision::PersistentId &a,
                       const vtr::vision::PersistentId &b) {
  return a.stamp == b.stamp && a.robot == b.robot;
}

// Equality comparison so it can be used in a sorted map
inline bool operator==(const vtr::vision::LandmarkId &a,
                       const vtr::vision::LandmarkId &b) {
  return a.persistent == b.persistent && a.rig == b.rig &&
         a.camera == b.camera && a.channel == b.channel && a.index == b.index;
}

// Inequality comparison so it can be used in a sorted map.
inline bool operator!=(const vtr::vision::PersistentId &a,
                       const vtr::vision::PersistentId &b) {
  return !(a == b);
}

// Inequality comparison so it can be used in a sorted map.
inline bool operator!=(const vtr::vision::LandmarkId &a,
                       const vtr::vision::LandmarkId &b) {
  return !(a == b);
}

// Inequality comparison so it can be used in a sorted map.
inline bool operator<(const vtr::vision::PersistentId &a,
                      const vtr::vision::PersistentId &b) {
  if (a.robot < b.robot)
    return true;
  if (a.robot > b.robot)
    return false;
  if (a.stamp < b.stamp)
    return true;
  if (a.stamp > b.stamp)
    return false;
  return false;  // equal
}

// Inequality comparison so it can be used in a sorted map.
inline bool operator>(const vtr::vision::PersistentId &a,
                      const vtr::vision::PersistentId &b) {
  if (a == b)
    return false;
  if (a < b)
    return false;
  return true;
}

// Inequality comparison so it can be used in a sorted map.
inline bool operator<(const vtr::vision::LandmarkId &a,
                      const vtr::vision::LandmarkId &b) {
  if (a.persistent < b.persistent)
    return true;
  if (a.persistent > b.persistent)
    return false;
  if (a.rig < b.rig)
    return true;
  if (a.rig > b.rig)
    return false;
  if (a.camera < b.camera)
    return true;
  if (a.camera > b.camera)
    return false;
  if (a.channel < b.channel)
    return true;
  if (a.channel > b.channel)
    return false;
  if (a.index < b.index)
    return true;
  if (a.index > b.index)
    return false;
  return false;  // equal
}

// Inequality comparison so it can be used in a sorted map.
inline bool operator>(const vtr::vision::LandmarkId &a,
                      const vtr::vision::LandmarkId &b) {
  return b < a && b != a;
}

struct LandmarkMatch {
  LandmarkId from;
  std::vector<LandmarkId> to;
};

typedef std::vector<LandmarkMatch> LandmarkMatches;

////////////////////////////////////////////////////////////////////////////////
// Landmarks

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Points3_t;
typedef Eigen::Matrix<double, 9, Eigen::Dynamic> Points9_t;

/// The feature landmarks observed by one channel
struct ChannelLandmarks {
  /// The channel name (e.g. grey, cc_desert, cc_forest)
  std::string name;
  /// The 3D point in the first camera's frame
  Points3_t points;
  /// The 3D point covariance (in the first camera's frame)
  Points9_t covariances;
  /// Appearence information (descriptor, precision, laplacian)
  Features appearance;
  /// Matches to previous experiences
  LandmarkMatches matches;
  /// Whether the landmark is valid
  std::vector<bool> valid;
};

/// The feature landmarks observed by a camera rig
struct RigLandmarks {
  /// The name of the rig (e.g. front-xb3, rear-visensor)
  std::string name;
  /// The landmarks associated with each channel
  std::vector<ChannelLandmarks> channels;
};

typedef std::vector<RigLandmarks> SuiteLandmarks;

////////////////////////////////////////////////////////////////////////////////

typedef cv::Point2f Point;
typedef std::vector<Point> Points;
typedef std::vector<float> Precisions;
typedef std::vector<Eigen::Matrix2d> Covariances;
// Observations
struct Observations {
  std::string name;
  Points points;
  Precisions precisions;
  Covariances covariances;
  LandmarkMatches landmarks;
};

struct ChannelObservations {
  std::string name;
  std::vector<Observations> cameras;
  bool fully_matched;
};

struct RigObservations {
  std::string name;
  std::vector<ChannelObservations> channels;
};

////////////////////////////////////////////////////////////////////////////////
// Bag Of Words

// A vocabulary is just a collection of LandmarkIds
typedef std::vector<LandmarkId> ChannelBowVocabulary;
typedef std::vector<ChannelBowVocabulary> RigBowVocabulary;
typedef std::vector<RigBowVocabulary> SuiteBowVocabulary;

// A BoW Descriptor is a map from LandmarkId to a count of how many times it was
// seen Note: zero counts should not be present, to keep things sparse and fast
typedef std::pair<LandmarkId, unsigned> BowWordCount;
typedef std::map<LandmarkId, unsigned> BowDescriptor;

////////////////////////////////////////////////////////////////////////////////
// Calibrations
typedef Eigen::Matrix<double, 5, 1> CameraDistortion;
typedef std::vector<CameraDistortion,
                    Eigen::aligned_allocator<CameraDistortion> >
    CameraDistortions;
typedef Eigen::Matrix<double, 3, 3> CameraIntrinsic;
typedef std::vector<CameraIntrinsic, Eigen::aligned_allocator<CameraIntrinsic> >
    CameraIntrinsics;
typedef Eigen::Matrix<double, 3, 4> CameraProjection;
typedef std::vector<CameraProjection,
                    Eigen::aligned_allocator<CameraProjection> >
    CameraProjections;
typedef lgmath::se3::Transformation Transform;
typedef std::vector<Transform, Eigen::aligned_allocator<Transform> > Transforms;

/// Rigid camera set information.
struct RigCalibration {
  /// Distortion parameters (k0, k1, p0, p1, k3), one for each camera in the
  /// rig.
  CameraDistortions distortions;
  /// Intrinsic camera matrices, one for each camera in the rig.
  CameraIntrinsics intrinsics;
  /** \brief Transform from origin point to each camera
   */
  Transforms extrinsics;
  /** \brief Indicates whether the rig is rectified (true) or more general
   * (false)
   */
  bool rectified;
};

/// IMU information.
struct IMUCalibration {
  /// Accelerometer and gyro biases
  Eigen::Matrix<double, 6, 1> bias;
  /// Transform from origin point to IMU
  Transform extrinsics;
};

/// Rigid camera set + IMU information.
struct InertialRigCalibration {
  /// The camera calibrations
  RigCalibration rig_calibration;
  /// The IMU calibrations
  IMUCalibration imu_calibration;
};

////////////////////////////////////////////////////////////////////////////////

}  // namespace vision
}  // namespace vtr

// custom specialization of std::hash can be injected in namespace std
namespace std {

// hash for feature ids
template <>
struct hash<vtr::vision::LandmarkId> {
  typedef vtr::vision::LandmarkId argument_type;
  typedef std::size_t result_type;
  result_type operator()(argument_type const &lid) const {
    // return std::hash(s.SerializeAsString()); // <- easy but slow
    result_type seed = 0;
    vtr::common::hash_combine(seed, lid.persistent.stamp, lid.persistent.robot,
                              lid.rig, lid.channel, /*lid.camera,*/ lid.index);
    return seed;
  }  // ()
};   // hash
}  // namespace std
