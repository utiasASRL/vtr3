#pragma once

#include <vtr_sensors/VtrSensor.hpp>
#include <vtr_sensors/Camera1394.hpp>

#include <dc1394/control.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <triclops.h>

#include <vector>

namespace vtr {
namespace sensors {
namespace xb3 {

/// @brief DC1394 video frame
struct DC1394Frame {      //todo: not sure if this is the right place for these

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
  
  /// @brief @brief The type of images in this channel (RGB, bayer, grey, colour-constant etc);
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
  bool sub_pixel_interpolation;

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
    float opticalCenterRow = 5;
    float opticalCenterCol = 6;
    float focalLength = 7;
    };

class BumblebeeXb3 : public VtrSensor {
 public:

  explicit BumblebeeXb3(Xb3Configuration config);

  ~BumblebeeXb3() = default;

 protected:

  vtr_messages::msg::RigImages grabSensorFrameBlocking() override;

  std::shared_ptr<DC1394Frame> grabFrameFromCamera();

  RigImages BayerToStereo(const std::shared_ptr<DC1394Frame> &raw_frame);

  void initializeCamera();

  void visualizeData() override;

  void publishData(vtr_messages::msg::RigImages image) override;

  RigImages rectifyStereo(RigImages images);

  void grabXB3Calibration();

  /// @brief The 1394 camera object.
  std::unique_ptr<Camera1394> camera_;

  Xb3Configuration xb3_config_;

  /// @brief the camera context
  TriclopsContext context_;

 private:

  /// @brief Maps resolutions to rectification matrix indices.
  std::map<std::pair<double,double>,int> rectification_map_;

  /// @brief   different for different resolutions
  std::vector<RectificationWarp> warp_;      //using this instead of full XB3Calibration for now

};

}  // namespace xb3
}  // namespace sensors
}  // namespace vtr