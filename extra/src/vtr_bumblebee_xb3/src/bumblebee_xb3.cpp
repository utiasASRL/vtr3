
#include <vtr_bumblebee_xb3/bumblebee_xb3.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace vtr {
namespace sensors {
namespace xb3 {

BumblebeeXb3::BumblebeeXb3(std::shared_ptr<rclcpp::Node> node,
                           Xb3Configuration config)
    : VtrSensor(std::move(node), "xb3_images"), xb3_config_(std::move(config)) {
  initializeCamera();

  calibration_srv_ = node_->create_service<GetRigCalibration>(
      "xb3_calibration",
      std::bind(&BumblebeeXb3::_calibrationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void BumblebeeXb3::_calibrationCallback(
    const std::shared_ptr<GetRigCalibration::Request>,
    std::shared_ptr<GetRigCalibration::Response> response) {
  response->calibration = calibration_msg_;
}

vtr_messages::msg::RigImages BumblebeeXb3::grabSensorFrameBlocking() {
  vtr_messages::msg::RigImages sensor_message;

  // Grab the image from the device.
  auto XB3Frame = grabFrameFromCamera();

  // Perform bayer conversion.
  auto raw_stereo_frame = BayerToStereo(XB3Frame);

  // Rectify the image.
  auto processed_stereo = rectifyStereo(raw_stereo_frame);

  // Set the stamp.
  for (auto &camera : processed_stereo.channels[0].cameras) {
    camera.nanoseconds_since_epoch = XB3Frame->timestamp * 1e3;
  }
  sensor_message.vtr_header.sensor_time_stamp.nanoseconds_since_epoch =
      XB3Frame->timestamp * 1e3;

  // Iterate over channels and cameras (typically only one channel, two cameras)
  for (const auto &channel : processed_stereo.channels) {
    vtr_messages::msg::ChannelImages chan_im;
    for (const auto &camera : channel.cameras) {
      vtr_messages::msg::Image cam_im;

      cam_im.height = camera.height;
      cam_im.width = camera.width;
      cam_im.encoding = camera.encoding;
      cam_im.is_bigendian = camera.is_bigendian;
      cam_im.stamp.nanoseconds_since_epoch = camera.nanoseconds_since_epoch;
      cam_im.step = camera.step;
      cam_im.data = camera.data;

      chan_im.cameras.push_back(cam_im);
    }
    sensor_message.channels.push_back(chan_im);
  }
  sensor_message.name = "front_xb3";

  return sensor_message;
}

std::shared_ptr<DC1394Frame> BumblebeeXb3::grabFrameFromCamera() {
  std::shared_ptr<dc1394video_frame_t> bayerFrame = camera_->getNextFrame();

  // Fill out the frame information
  auto rawFrame = std::make_shared<DC1394Frame>();
  rawFrame->color_coding = bayerFrame->color_coding;
  rawFrame->data_depth = bayerFrame->data_depth;
  rawFrame->data_in_padding = bayerFrame->data_in_padding;
  rawFrame->frames_behind = bayerFrame->frames_behind;
  rawFrame->id = bayerFrame->id;
  rawFrame->image_bytes = bayerFrame->image_bytes;
  rawFrame->little_endian = bayerFrame->little_endian;
  rawFrame->packet_size = bayerFrame->packet_size;
  rawFrame->packets_per_frame = bayerFrame->packets_per_frame;
  rawFrame->padding_bytes = bayerFrame->padding_bytes;
  rawFrame->horizontal_position = bayerFrame->position[0];
  rawFrame->vertical_position = bayerFrame->position[1];
  rawFrame->width = bayerFrame->size[0];
  rawFrame->height = bayerFrame->size[1];
  rawFrame->stride = bayerFrame->stride;
  rawFrame->timestamp = bayerFrame->timestamp;
  rawFrame->total_bytes = bayerFrame->total_bytes;
  rawFrame->video_mode = bayerFrame->video_mode;
  rawFrame->yuv_byte_order = bayerFrame->yuv_byte_order;

  // Move the bytes over.
  std::vector<char> im(bayerFrame->image,
                       bayerFrame->image + bayerFrame->total_bytes);
  rawFrame->image = im;

  return rawFrame;
}

RigImages BumblebeeXb3::BayerToStereo(
    const std::shared_ptr<DC1394Frame> &bayer_frame) const {
  const auto &raw_frame = *bayer_frame;

  // Deinterlace the stereo images from the xb3
  std::array<cv::Mat, 2> deinterlaced{
      cv::Mat(raw_frame.height, raw_frame.width, CV_8UC1),
      cv::Mat(raw_frame.height, raw_frame.width, CV_8UC1)};

  // left/right image indexes for XB3
  unsigned lidx = 0;
  unsigned ridx = 1;

  int imageSize = 1280 * 960;
  const char *p = &raw_frame.image[0];
  const char *end = p + imageSize * 2;
  char *l = (char *)&deinterlaced[lidx].data[0];
  char *r = (char *)&deinterlaced[ridx].data[0];
  while (p < end) {
    *(r++) = *(p++);
    *(l++) = *(p++);
  }

  struct ChannelInfo {
    struct CamInfo {
      std::string name;
      cv::Mat wrapper;
    };
    std::string name;
    std::string encoding;
    unsigned char depth;
    decltype(CV_8UC3) cv_type;
    int source_chan;
    decltype(cv::COLOR_BayerGB2RGB) cv_convert;
    std::array<CamInfo, 2> cam;
  };

  // figure out the colour conversion type
  decltype(cv::COLOR_BayerGB2RGB) conversion_type = cv::COLOR_BayerGB2RGB;
  std::vector<ChannelInfo> chan_infos;
  chan_infos.push_back({"RGB",
                        "bgr8",
                        3,
                        CV_8UC3,
                        -1,
                        conversion_type,
                        {{{"left", cv::Mat()}, {"right", cv::Mat()}}}});
  if (xb3_config_.output_gray) {
    chan_infos.push_back({"grayscale",
                          "mono8",
                          1,
                          CV_8UC1,
                          0,
                          cv::COLOR_RGB2GRAY,
                          {{{"left", cv::Mat()}, {"right", cv::Mat()}}}});
  }

  // fill in the rig message
  RigImages output_rig;
  output_rig.name = xb3_config_.camera_name;

  // first loop over all the channels
  for (unsigned chan_i = 0; chan_i < chan_infos.size(); ++chan_i) {
    auto &chan_info_i = chan_infos[chan_i];

    // add a new named channel
    ChannelImages chan;
    chan.name = chan_info_i.name;

    // loop over all the cameras
    for (unsigned cam_i = 0; cam_i < chan_info_i.cam.size(); ++cam_i) {
      auto &cam_info_i = chan_info_i.cam[cam_i];

      // add a new camera
      Image cam;

      // set metadata
      cam.name = cam_info_i.name;
      cam.height = raw_frame.height;
      cam.width = raw_frame.width;
      cam.depth = chan_info_i.depth;
      cam.step = raw_frame.width * cam.depth;
      cam.encoding = chan_info_i.encoding;

      // create room for the image to be stored
      // std::vector<unsigned char> data;
      auto &data = cam.data;

      data.resize(raw_frame.height * raw_frame.width * chan_info_i.depth);
      auto &wrapper = chan_info_i.cam[cam_i].wrapper;
      wrapper = cv::Mat(cam.height, cam.width, chan_info_i.cv_type,
                        (void *)data.data());

      // convert the image from another channel (or raw)
      cv::Mat &src =
          chan_info_i.source_chan < 0
              ? deinterlaced[cam_i]
              : chan_infos[chan_info_i.source_chan].cam[cam_i].wrapper;
      cv::cvtColor(src, wrapper, chan_info_i.cv_convert);

      // possibly show the channel
      if (xb3_config_.show_raw_images)
        cv::imshow(cam_info_i.name + "-" + chan_info_i.name, wrapper);

      chan.cameras.push_back(cam);
    }
    output_rig.channels.push_back(chan);
  }
  return output_rig;
}

void BumblebeeXb3::initializeCamera() {
  Camera1394::Config camera_config{};
  camera_config.numDmaBuffers_ = 4;
  camera_config.transmissionStatusRetries_ = 4;

  if (xb3_config_.packet_multiplier == 0) {
    xb3_config_.packet_multiplier = DC1394_USE_MAX_AVAIL;
  }

  camera_config.captureMode_ = STEREO_WIDE;
  camera_config.busSpeed_ = BUS_800;
  camera_config.raw_resolution_ = RES_1280x960;
  camera_config.packetSize_ = DC1394_USE_MAX_AVAIL;

  camera_ = std::make_unique<Camera1394>(camera_config);
  camera_->init();
  camera_->start();

  TriclopsError error;

  // initialize Triclops
  printf("Loading PGR Triclops\n");
  error = triclopsGetDefaultContextFromFile(
      &context_, const_cast<char *>(camera_->calibrationFile().c_str()));
  if (error != TriclopsErrorOk) {
    printf("Can't open calibration file %s \n",
           camera_->calibrationFile().c_str());
    exit(1);
  }

  // make sure we are in subpixel mode
  error = triclopsSetSubpixelInterpolation(context_, 1);
  if (error != TriclopsErrorOk) {
    printf("Cannot set subpixel interpolation\n");
    exit(1);
  }

  // setup config
  TriclopsCameraConfiguration config = TriCfg_2CAM_HORIZONTAL_WIDE;

  // Set the camera configuration
  error = triclopsSetCameraConfiguration(context_, config);
  if (error != TriclopsErrorOk) {
    printf("Cannot set config mode!\n");
    exit(1);
  }

  xb3_calibration_ = grabXB3Calibration();
  calibration_msg_ = generateRigCalibration();

  triclopsSetDoStereo(context_, false);
  // As of April 13, 2011, the Triclops library crashes if the thread count
  // isn't set to 1
  triclopsSetMaxThreadCount(context_, 1);
}

void BumblebeeXb3::publishData(vtr_messages::msg::RigImages image) {
  sensor_pub_->publish(image);
}

RigImages BumblebeeXb3::rectifyStereo(const RigImages &raw_image) {
  RigImages output_image;
  output_image.name = raw_image.name;

  auto height = xb3_config_.rectified_image_size.height;
  auto width = xb3_config_.rectified_image_size.width;
  auto warp_idx = rectification_map_[std::pair<double, double>{height, width}];
  auto warp = warp_[warp_idx];

  for (const auto &channel : raw_image.channels) {
    ChannelImages output_channel;

    output_channel.name = channel.name;
    for (const auto &camera : channel.cameras) {
      Image output_camera;

      output_camera.name = camera.name;
      output_camera.height = xb3_config_.rectified_image_size.height;
      output_camera.width = xb3_config_.rectified_image_size.width;
      output_camera.step = xb3_config_.rectified_image_size.width;
      output_camera.encoding = camera.encoding;

      auto &raw_data_string = camera.data;

      int outputmode = -1;
      int datasize = 0;
      if (camera.encoding == "bgr8") {
        datasize = height * width * 3;
        outputmode = CV_8UC3;
      } else if (camera.encoding == "mono8") {
        datasize = height * width;
        outputmode = CV_8UC1;
      }

      auto &output_data = output_camera.data;
      output_data.resize(datasize);

      cv::Mat cv_raw = cv::Mat(camera.height, camera.width, outputmode,
                               (void *)raw_data_string.data());
      cv::Mat cv_rect =
          cv::Mat(height, width, outputmode, (void *)output_data.data());

      // output_camera not pushed back to output_channel until bottom of loop so
      // size will be 0 for left, 1 for right
      if (output_channel.cameras.empty()) {
        cv::Mat leftMapCols =
            cv::Mat(height, width, CV_32FC1,
                    (void *)warp.left_rectification_matrix_cols.data());
        cv::Mat leftMapRows =
            cv::Mat(height, width, CV_32FC1,
                    (void *)warp.left_rectification_matrix_rows.data());
        cv::remap(cv_raw, cv_rect, leftMapCols, leftMapRows, cv::INTER_LINEAR);
      } else if (output_channel.cameras.size() == 1) {
        cv::Mat rightMapCols =
            cv::Mat(height, width, CV_32FC1,
                    (void *)warp.right_rectification_matrix_cols.data());
        cv::Mat rightMapRows =
            cv::Mat(height, width, CV_32FC1,
                    (void *)warp.right_rectification_matrix_rows.data());
        cv::remap(cv_raw, cv_rect, rightMapCols, rightMapRows,
                  cv::INTER_LINEAR);
      } else {
        throw std::runtime_error("More than two cameras found (?)\n");
      }

      if (xb3_config_.show_rectified_images) {
        cv::imshow(channel.name + "/" + camera.name + "/rectified", cv_rect);
      }

      output_channel.cameras.push_back(output_camera);
    }
    output_image.channels.push_back(output_channel);
  }
  return output_image;
}

vtr_messages::msg::XB3CalibrationResponse BumblebeeXb3::grabXB3Calibration() {
  std::array<cv::Size, 5> resolutions{cv::Size(1280, 960), cv::Size(1024, 768),
                                      cv::Size(640, 480), cv::Size(512, 384),
                                      cv::Size(320, 240)};

  float focalLength;
  // Iterate through the resolutions.
  for (uint32_t idx = 0; idx < resolutions.size(); ++idx) {
    // set the resolution of the camera
    int raw_image_height = DEFAULT_RAW_IMAGE_HEIGHT;
    int raw_image_width = DEFAULT_RAW_IMAGE_WIDTH;

    auto error = triclopsSetResolutionAndPrepare(
        context_, resolutions[idx].height, resolutions[idx].width,
        raw_image_height, raw_image_width);
    if (error != TriclopsErrorOk) {
      printf("Cannot set resolution and prepare !\n");
      exit(1);
    }

    // Get this resolution's optical center

    // Set up the rectification data in the map.
    RectificationWarp warp;
    float opticalCenterRow;
    float opticalCenterCol;
    triclopsGetImageCenter(context_, &opticalCenterRow, &opticalCenterCol);
    warp.opticalCenterRow = opticalCenterRow;
    triclopsGetFocalLength(context_, &focalLength);
    warp.opticalCenterCol = opticalCenterCol;
    warp.focalLength = focalLength;
    int pixelIdx = 0;
    auto &left_rows = warp.left_rectification_matrix_rows;
    auto &left_cols = warp.left_rectification_matrix_cols;
    auto &right_rows = warp.right_rectification_matrix_rows;
    auto &right_cols = warp.right_rectification_matrix_cols;
    auto datasize =
        resolutions[idx].height * resolutions[idx].width * sizeof(float);
    left_rows.resize(datasize);
    left_cols.resize(datasize);
    right_rows.resize(datasize);
    right_cols.resize(datasize);

    auto *left_rows_data = (float *)left_rows.data();
    auto *left_cols_data = (float *)left_cols.data();
    auto *right_rows_data = (float *)right_rows.data();
    auto *right_cols_data = (float *)right_cols.data();
    for (int row = 0; row < resolutions[idx].height; ++row) {
      for (int col = 0; col < resolutions[idx].width; ++col) {
        triclopsUnrectifyPixel(context_, TriCam_LEFT, row, col,
                               left_rows_data++, left_cols_data++);

        triclopsUnrectifyPixel(context_, TriCam_RIGHT, row, col,
                               right_rows_data++, right_cols_data++);
        pixelIdx++;
      }
    }

    warp_.push_back(warp);
    rectification_map_.insert(
        {std::pair<double, double>{resolutions[idx].height,
                                   resolutions[idx].width},
         idx});
  }

  // Obtain Camera information
  int serialNumber;
  float baseline;
  triclopsGetBaseline(context_, &baseline);
  triclopsGetSerialNumber(context_, &serialNumber);

  printf("Serial Number %i\n", serialNumber);
  printf("Baseline: %f\n", baseline);

  vtr_messages::msg::XB3CalibrationResponse XB3Response;
  XB3Response.baseline = baseline;
  XB3Response.serial_number = serialNumber;

  // set the Calibration messages up.
  auto warp_idx = rectification_map_[std::pair<double, double>{
      xb3_config_.rectified_image_size.height,
      xb3_config_.rectified_image_size.width}];
  auto warp = warp_[warp_idx];
  XB3Response.rectified_height = xb3_config_.rectified_image_size.height;
  XB3Response.rectified_width = xb3_config_.rectified_image_size.width;
  XB3Response.optical_center_row = warp.opticalCenterRow;
  XB3Response.optical_center_col = warp.opticalCenterCol;
  XB3Response.focal_length = warp.focalLength;
  return XB3Response;
}

vtr_messages::msg::RigCalibration BumblebeeXb3::generateRigCalibration() {
  auto rig_calibration = vtr_messages::msg::RigCalibration();
  rig_calibration.rectified = true;

  // set up rig calibration as identity
  // set up the extrinsic
  vtr_messages::msg::Transform left_extrinsics;
  left_extrinsics.translation.x = 0;
  left_extrinsics.translation.y = 0;
  left_extrinsics.translation.z = 0;
  left_extrinsics.orientation.x = 0;
  left_extrinsics.orientation.y = 0;
  left_extrinsics.orientation.z = 0;
  rig_calibration.extrinsics.push_back(left_extrinsics);

  // Copy data to right extrinsic, translate by the baseline.
  auto &right_extrinsics = left_extrinsics;
  right_extrinsics.translation.x = -xb3_calibration_.baseline;
  rig_calibration.extrinsics.push_back(right_extrinsics);

  // Fill out intrinsics
  vtr_messages::msg::CameraCalibration left_intrinsics;
  for (int idx = 0; idx < 9; ++idx) {
    left_intrinsics.k_mat.push_back(0);
  }

  left_intrinsics.k_mat[0] = xb3_calibration_.focal_length;
  left_intrinsics.k_mat[2] = xb3_calibration_.optical_center_col;
  left_intrinsics.k_mat[4] = xb3_calibration_.focal_length;
  left_intrinsics.k_mat[5] = xb3_calibration_.optical_center_row;
  left_intrinsics.k_mat[8] = 1.0;

  left_intrinsics.distortion_model = "";

  rig_calibration.intrinsics.push_back(left_intrinsics);
  auto &right_intrinsics = left_intrinsics;
  rig_calibration.intrinsics.push_back(right_intrinsics);

  return rig_calibration;
}

void BumblebeeXb3::visualizeData() {
  // cv::imshow() called in BayerToStereo() and rectifyStereo(), this lets them
  // be seen
  if (xb3_config_.show_raw_images || xb3_config_.show_rectified_images) {
    cv::waitKey(1);
  }
}

}  // namespace xb3
}  // namespace sensors
}  // namespace vtr