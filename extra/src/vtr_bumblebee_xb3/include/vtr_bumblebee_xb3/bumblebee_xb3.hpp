#pragma once

#include <vector>

#include <dc1394/control.h>
#include <triclops.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vtr_bumblebee_xb3/camera1394.hpp>
#include <vtr_bumblebee_xb3/vtr_sensor.hpp>

#include <vtr_messages/msg/rig_image_calib.hpp>

#include <vtr_messages/msg/xb3_calibration_response.hpp>


namespace vtr {
namespace sensors {
namespace xb3 {

/// @brief DC1394 video frame
struct DC1394Frame {
  /// @brief Image Frame
  /// @details May contain padding data too (vendor specific)
  std::vector<char> image;

  /// @brief Image height (pixels)
  int32_t height;

  /// @brief Image width (pixels)
  int32_t width;

  /// @brief WOI/ROI horizontal position
  int32_t horizontal_position;

  /// @brief WOI/ROI vertical position
  int32_t vertical_position;

  /// @brief The color coding of the image
  /// @details Valid values are...
  ///     MONO8   = 352
  ///     YUV411  = 353
  ///     YUV422  = 354
  ///     YUV444  = 355
  ///     RGB8    = 356
  ///     MONO16  = 357
  ///     RGB16   = 358
  ///     MONO16S = 359
  ///     RGB16S  = 360
  ///     RAW8    = 361
  ///     RAW16   = 362
  int32_t color_coding;

  /// @brief The color filter of the image
  /// @details Valid values are...
  ///     RGGB = 512
  ///     GBRG = 513
  ///     GRBG = 514
  ///     GRBG = 515
  ///     BGGR = 516
  int32_t color_filter;

  int32_t yuv_byte_order;
  int32_t data_depth;
  int32_t stride;
  int32_t video_mode;
  int64_t total_bytes;
  int32_t image_bytes;
  int32_t padding_bytes;
  int32_t packet_size;
  int32_t packets_per_frame;
  int64_t timestamp;
  int32_t frames_behind;
  int32_t id;
  int64_t allocated_image_bytes;
  bool little_endian;
  bool data_in_padding;
};

struct Image {
  uint64_t nanoseconds_since_epoch;

  /// @brief Image Height, pixels
  int32_t height;

  /// @brief Image Width, pixels
  int32_t width;

  /// @brief Image encoding
  std::string encoding;

  /// @brief The pixel depth (how many bytes per pixel)
  int32_t depth;

  /// @brief Image Step
  /// @details total size of row, in bytes
  int32_t step;

  /// @brief Image Data
  /// @details The byte array of size width * step
  std::vector<unsigned char> data;

  /// @brief Big endian flag
  bool is_bigendian;

  /// @brief The name of the physical camera (e.g left, right etc).
  std::string name;
};

struct ChannelImages {
  /// @brief The images for each physical camera (i.e. left/right etc.)
  std::vector<Image> cameras;

  /// @brief @brief The type of images in this channel (RGB, bayer, grey,
  /// colour-constant etc);
  std::string name;
};

struct RigImages {
  /// @brief The images for each channel
  std::vector<ChannelImages> channels;

  /// @brief The name of the rig to assist in URDF lookup.
  std::string name;
};

struct Xb3Configuration {
  /// @brief
  std::string camera_model;
  std::string camera_name;
  uint32_t packet_multiplier;
  cv::Size rectified_image_size;
  bool output_gray;

  bool show_raw_images;
  bool show_rectified_images;
};

struct RectificationWarp {
  std::vector<unsigned char> left_rectification_matrix_rows;
  std::vector<unsigned char> left_rectification_matrix_cols;
  std::vector<unsigned char> right_rectification_matrix_rows;
  std::vector<unsigned char> right_rectification_matrix_cols;
  float opticalCenterRow;
  float opticalCenterCol;
  float focalLength;
};

class BumblebeeXb3 : public VtrSensor<vtr_messages::msg::RigImageCalib, vtr_messages::msg::RigImages, vtr_messages::msg::RigCalibration> {
//  class BumblebeeXb3 : public VtrSensor<vtr_messages::msg::RigImageCalib> {

 public:
  BumblebeeXb3(std::shared_ptr<rclcpp::Node> node, Xb3Configuration config);

  ~BumblebeeXb3() = default;

 protected:

  /// @brief Gets frame from camera, deBayerizes and rectifies
  vtr_messages::msg::RigImageCalib grabSensorFrameBlocking() override;

  /// @brief Gets Bayer image from XB3
  std::shared_ptr<DC1394Frame> grabFrameFromCamera();

  /// @brief Converts Bayer image to RGB
  [[nodiscard]] RigImages BayerToStereo(const std::shared_ptr<DC1394Frame> &raw_frame) const;

  /// @brief Sets camera calibration/configuration. Uses Triclops library
  void initializeCamera();

  /// @brief Displays images if requested in parameter file
  void visualizeData() override;

  /// @brief Publishes a custom ROS2 RigImages message on images topic
  void publishData(vtr_messages::msg::RigImageCalib image) override;

  /// @brief Uses camera calibration to rectify, resize image
  RigImages rectifyStereo(const RigImages &images);

  /// @brief Gets calibration parameters from XB3 unit connected to computer
  vtr_messages::msg::XB3CalibrationResponse grabXB3Calibration();

  /// @brief Generates calibration_msg_ from xb3_calibration_
  vtr_messages::msg::RigCalibration generateRigCalibration();

  /// @brief The 1394 camera object.
  std::unique_ptr<Camera1394> camera_;

  /// @brief Camera parameters
  Xb3Configuration xb3_config_;

  /// @brief the camera context
  TriclopsContext context_{};

 private:
  /// \brief  Provide this camera's calibration when requested
  // void _calibrationCallback(
  //     const std::shared_ptr<GetRigCalibration::Request> request,
  //     std::shared_ptr<GetRigCalibration::Response> response);

  /// @brief Maps resolutions to rectification matrix indices.
  std::map<std::pair<double, double>, int> rectification_map_;

  /// @brief   different for different resolutions
  std::vector<RectificationWarp> warp_;

  /// @brief Parameters specific to the XB3 camera being used
  vtr_messages::msg::XB3CalibrationResponse xb3_calibration_;

  /// @brief Calibration for the stereo camera to be stored with dataset
  vtr_messages::msg::RigCalibration calibration_msg_;

  /// @brief ROS2 service providing camera's calibration
  // rclcpp::Service<GetRigCalibration>::SharedPtr calibration_srv_;
};

}  // namespace xb3
}  // namespace sensors
}  // namespace vtr