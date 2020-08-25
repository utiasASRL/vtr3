
#include <vtr_sensors/BumblebeeXb3.hpp>

#include <utility>
#include <vector>

namespace vtr {
namespace sensors {
namespace xb3 {

BumblebeeXb3::BumblebeeXb3(Xb3Configuration config) : xb3_config_(std::move(config)) {

  initializeCamera();
}

sensor_msgs__msg__Image BumblebeeXb3::grabSensorFrameBlocking() {

  sensor_msgs__msg__Image sensor_message;

//todo
  // Grab the image from the device.
  auto XB3Frame = grabFrameFromCamera();

  // Perform bayer conversion.
  auto raw_stereo_frame = BayerToStereo(XB3Frame);

#if 0
  // Rectify the image.   todo
  auto processed_stereo = rectifyStereo(raw_stereo_frame);
#else
  auto processed_stereo = raw_stereo_frame;  //just copying unrectified image for now
#endif

  // Set the stamp.
  for(auto camera : processed_stereo.channels[0].cameras) {
    camera.nanoseconds_since_epoch = XB3Frame->timestamp * 1e3;
  }

  return sensor_message;
}


std::shared_ptr<DC1394Frame> BumblebeeXb3::grabFrameFromCamera() {


  std::shared_ptr<dc1394video_frame_t>  bayerFrame = camera_->getNextFrame();

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
  //auto start_add = bayerFrame->image;
  //auto deb = bayerFrame->image;
  //std::vector<char> w_(w,w+len);

  std::vector<char> im(bayerFrame->image,bayerFrame->image + bayerFrame->total_bytes);
  //std::vector<char> im(bayerFrame->image[0],bayerFrame->image[0] + bayerFrame->total_bytes);
  //std::vector<char> im(bayerFrame->total_bytes ,bayerFrame->image[0]);
  rawFrame->image = im;
  //rawFrame->image.assign(bayerFrame->image, bayerFrame->total_bytes);

  return rawFrame;

}


RigImages BumblebeeXb3::BayerToStereo(const std::shared_ptr<DC1394Frame> &bayer_frame) {
  const auto &raw_frame = *bayer_frame;

  // Deinterlace the stereo images from the xb3
  std::array<cv::Mat,2> deinterlaced {
      cv::Mat(raw_frame.height,raw_frame.width,CV_8UC1),
      cv::Mat(raw_frame.height,raw_frame.width,CV_8UC1) };

  // left/right image indexes for XB3
  unsigned lidx = 0;
  unsigned ridx = 1;

  // for some reason, the BB2 left/right images are swapped
  if(xb3_config_.camera_model == "BB2") {
    lidx = 1;
    ridx = 0;
  }

  int imageSize = (xb3_config_.camera_model == "BB2") ? 640*480: 1280*960; // TODO raw_frame.height() * raw_frame.width() ?
  const char * p = &raw_frame.image[0];
  const char * end = p + imageSize * 2;
  char *l = (char*)&deinterlaced[lidx].data[0];
  char *r = (char*)&deinterlaced[ridx].data[0];
  while (p < end) {
    *(r++) = *(p++);
    *(l++) = *(p++);
  }


  // Figure out which channels we'll output
  // TODO we could output multiple channels,
  // right now just one.
  struct ChannelInfo {
    struct CamInfo {
      std::string name;
      cv::Mat wrapper;
    };
    std::string name;
    std::string encoding;
    char depth;
    decltype(CV_8UC3) cv_type;
    int source_chan;
    decltype(cv::COLOR_BayerGB2RGB) cv_convert;
    std::array<CamInfo,2> cam;
  };

  // figure out the colour conversion type
  decltype(cv::COLOR_BayerGB2RGB) conversion_type = cv::COLOR_BayerGB2RGB;
  if(xb3_config_.camera_model == "BB2") {
    conversion_type = cv::COLOR_BayerBG2RGB;
  }
  std::vector<ChannelInfo> chan_infos;
  chan_infos.push_back({"RGB", "bgr8", 3, CV_8UC3, -1,
                        conversion_type, {{{"left"}, {"right"}}}});
  if (xb3_config_.output_gray) {
    chan_infos.push_back({"grayscale", "mono8", 1, CV_8UC1, 0,
                          cv::COLOR_RGB2GRAY, {{{"left"}, {"right"}}}});
  }

  // fill in the rig message
  RigImages output_rig;
  output_rig.name = xb3_config_.camera_name;

  // first loop over all the channels
  for (unsigned chan_i = 0; chan_i < chan_infos.size(); ++chan_i) {
    auto & chan_info_i = chan_infos[chan_i];

    // add a new named channel
    ChannelImages chan;
    chan.name = chan_info_i.name;

    // loop over all the cameras
    for (unsigned cam_i = 0; cam_i < chan_info_i.cam.size(); ++cam_i) {
      auto & cam_info_i = chan_info_i.cam[cam_i];

      // add a new camera
      Image cam;

      // set metadata
      cam.name = cam_info_i.name;
      cam.height = raw_frame.height;
      cam.width = raw_frame.width;
      cam.depth = chan_info_i.depth;
      cam.step = raw_frame.width*cam.depth;
      cam.encoding = chan_info_i.encoding;

      // create room for the image to be stored
      std::string data;

      data.resize(raw_frame.height * raw_frame.width * chan_info_i.depth);
      auto & wrapper = chan_info_i.cam[cam_i].wrapper;
      wrapper = cv::Mat(cam.height,cam.width,chan_info_i.cv_type,(void*)data.data());

      // convert the image from another channel (or raw)
      cv::Mat & src = chan_info_i.source_chan < 0 ?
                      deinterlaced[cam_i] :
                      chan_infos[chan_info_i.source_chan].cam[cam_i].wrapper;
      cv::cvtColor(src,wrapper,chan_info_i.cv_convert);

      // possibly show the channel
      if(xb3_config_.show_raw_images)
        cv::imshow(cam_info_i.name+"-"+chan_info_i.name, wrapper);

      chan.cameras.push_back(cam);
    }
    output_rig.channels.push_back(chan);
  }
  return output_rig;
}

void BumblebeeXb3::initializeCamera() {

  Camera1394::Config camera_config;
  camera_config.numDmaBuffers_ = 4;
  camera_config.transmissionStatusRetries_ = 4;

  if(xb3_config_.packet_multiplier == 0) {
    xb3_config_.packet_multiplier = DC1394_USE_MAX_AVAIL;
  }

  // packetsize to get rate is weird
  camera_config.packetSize_ = (2*
      xb3_config_.rectified_image_size.width*
      xb3_config_.rectified_image_size.height*
      xb3_config_.packet_multiplier)/8000;

  if(xb3_config_.camera_model == "BB2") {
    camera_config.captureMode_ = STEREO_NARROW;
    camera_config.busSpeed_ = BUS_400;
    camera_config.raw_resolution_ = RES_640x480;
  } else {
    camera_config.captureMode_ = STEREO_WIDE;
    camera_config.busSpeed_ = BUS_800;
    camera_config.raw_resolution_ = RES_1280x960;
    camera_config.packetSize_ = DC1394_USE_MAX_AVAIL;

  }
  camera_.reset(new Camera1394(camera_config));
  camera_->init();
  camera_->start();
  // Setup triclops variables.
  TriclopsImage       depthImage;
  TriclopsInput       inputData;
  TriclopsError       error;

  // initialize triclops
  printf("loading pgr triclops\n");
  error = triclopsGetDefaultContextFromFile(&context_,
                                            const_cast<char*>(camera_->calibrationFile().c_str()));
  if ( error != TriclopsErrorOk )
  {
    printf( "Can't open calibration file %s \n",camera_->calibrationFile().c_str());
    exit( 1 );
  }

  // make sure we are in subpixel mode
  error = triclopsSetSubpixelInterpolation(context_,1);
  if ( error != TriclopsErrorOk )
  {
    printf( "Cannot set subpixel interpolation\n" );
    exit( 1 );
  }

  // setup config
  // TODO: Set this from request
  TriclopsCameraConfiguration config;

  if(xb3_config_.camera_model == "BB2") {
    config = TriCfg_2CAM_HORIZONTAL_NARROW;
  } else {
    config = TriCfg_2CAM_HORIZONTAL_WIDE;

  }
  // Set the camera configuration
  error = triclopsSetCameraConfiguration(context_,config);
  if ( error != TriclopsErrorOk )
  {
    printf( "Cannot set config mode!\n" );
    exit( 1 );
  }
#if 0
  LOG(INFO) << "Setting calibration";
#endif
#if 0
  xb3_calibration_ = grabXB3Calibration();
  rig_calibration_ = generateRigCalibration();
#endif
  triclopsSetDoStereo(context_,false);
  // As of April 13, 2011, the triclops library crashes if the thread count isn't set to 1
  triclopsSetMaxThreadCount(context_,1);
}


}  // namespace xb3
}  // namespace sensors
}  // namespace vtr