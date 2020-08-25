#pragma once

#include <memory>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <cassert>
#include <iostream>
#include <functional>

#include <vtr_sensors/Camera1394.hpp>
#include <vtr_sensors/assert_macros.hpp>
#include <vtr_sensors/Dc1394SafeCall.hpp>




//////////////////////////////////////////////////////////////
// Camera1394
//////////////////////////////////////////////////////////////
Camera1394::Camera1394() :
      camera_(0),
      initialized_(false),
      isCapturing_(false),
      isMonochrome_(false)
{
    config_.captureMode_ = STEREO_WIDE;
    config_.numDmaBuffers_ = 4;
    config_.busSpeed_ = BUS_800;
    config_.transmissionStatusRetries_ = 4;
    config_.raw_resolution_ = RES_1280x960;
    config_.packetSize_ = DC1394_USE_MAX_AVAIL;
}

Camera1394::Camera1394(Config config) :
      camera_(0),
      initialized_(false),
      isCapturing_(false),
      isMonochrome_(false),
      config_(config)
{

}

//////////////////////////////////////////////////////////////
// ~Camera1394
//////////////////////////////////////////////////////////////
Camera1394::~Camera1394()
{
  cleanUp();
}

//////////////////////////////////////////////////////////////
// cleanUp
//////////////////////////////////////////////////////////////
void Camera1394::cleanUp()
{
    if(camera_ != nullptr) {
        stop();
        dc1394_camera_free( camera_ );
        camera_ = nullptr;
    }
}

//////////////////////////////////////////////////////////////
// stop
//////////////////////////////////////////////////////////////
void Camera1394::stop()
{
    assert(camera_ != nullptr);
    if(isCapturing_) {
        dc1394_capture_stop( camera_ );
        dc1394_video_set_transmission( camera_, DC1394_OFF );
        isCapturing_ = false;
    }
}

void Camera1394::saveCalibrationFile(const std::string & path)
    {
      std::cout << "Attempting to save file to: " << path;
      const uint32_t bytesPerRegister = 4;
      uint32_t calibrationFileSize = 0;

      dc1394SafeCall("Getting the size of the calibration file",
		     dc1394_get_control_register,
		     camera_,
		     CONFIG_LENGTH_REGISTER, &calibrationFileSize,
		     ASRL_SOURCE_FILE_POS);

      assert(calibrationFileSize > 0); //, "Calibration file size was zero");
      std::ofstream fout(path.c_str(),std::ios::binary);
        assert(fout.good()); // ,"Unable to open file " << path << " for writing");


      uint32_t bytes = 0;
      char * chars = 0;
      for( uint32_t registerOffset = 0 ; registerOffset < calibrationFileSize; registerOffset += bytesPerRegister )
	{
	  dc1394SafeCall("Grab 4 bytes of the configuration file from the camera",
			 dc1394_get_control_register,
			 camera_,
			 CONFIG_DATA_REGISTER + registerOffset,
			 &bytes,
			 ASRL_SOURCE_FILE_POS);
	  // Write the bytes backwards to flip the endianness.
	  chars = reinterpret_cast<char*>(&bytes);
	  std::swap(chars[0],chars[3]);
	  std::swap(chars[1],chars[2]);
	  fout.write(chars,bytesPerRegister);
	}
}

void Camera1394::init()
{
    dc1394camera_list_t * rawlist = NULL;
    unsigned int nThisCam;

    // This never gets destroyed. It makes me uneasy but none of the sample code
    // I have found destroys this. Furthermore, the help for the destruction call
    // says: "dc1394_free() Liberates a context. Last function to use in your program.
    // After this, no libdc1394 function can be used." I suppose it should be some
    // kind of singleton. For now I'll just do wat the sample code does.
    dc1394_t * d = dc1394_new();

    // Enumerate cameras connected to the PC
    dc1394SafeCall("Enumerating cameras failed. Please check \n"
         "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
         "  - if you have read/write access to /dev/raw1394\n\n",
         dc1394_camera_enumerate,d, &rawlist, ASRL_SOURCE_FILE_POS);

    // Make sure the list gets cleaned up.
    std::shared_ptr<dc1394camera_list_t> list(rawlist,&dc1394_camera_free_list);
    printf("list size: %i \n",list->num);
    //  get the camera nodes and describe them as we find them
    if(list->num == 0) {
      throw std::runtime_error("Failed to find any cameras!!!!\n");
      return;
    }
    // Identify cameras. Use the first stereo camera that is found
    for ( nThisCam = 0; nThisCam < list->num; nThisCam++ ) {
        camera_ = dc1394_camera_new(d, list->ids[nThisCam].guid);

        if(!camera_) {
            std::cerr << "Failed to initialize camera with guid " << std::hex <<  list->ids[nThisCam].guid << std::dec << std::endl;
            continue;
        }


        std::string model = camera_->model;
        if ( model.substr(0,XB3_MODEL_NAME.size()) == XB3_MODEL_NAME ||
             model.substr(0,BB2_MODEL_NAME.size()) == BB2_MODEL_NAME) {

          dc1394SafeCall( "Get the camera serial number",
	              dc1394_get_control_register,
	              camera_, SERIAL_NUMBER_REGISTER,
	              &serialNumber_, ASRL_SOURCE_FILE_POS);

          std::cout << "Using camera with guid " << std::hex << list->ids[nThisCam].guid << std::dec << "\n" ;
          break;
        }

        dc1394_camera_free(camera_);
    } // for end (nthiscam)

    assert(nThisCam < list->num);

    // Color or B&W
    // Page 78/79 of the Point Grey Digital Camera Register Reference Manual
    uint32_t sensorInfo;
    dc1394SafeCall( "Get the camera sensor type",
          dc1394_get_control_register,
          camera_, SENSOR_INFO_REGISTER,
          &sensorInfo,ASRL_SOURCE_FILE_POS);
    isMonochrome_ = ( (sensorInfo & 0xf) == 0xf );
    std::cout << "The camera is " << (isMonochrome_? "monochrome" : "color") << std::endl;

    if(!isMonochrome_) {
        // Should we get the Bayer pattern of the camera?
        // Register 0x1040. Page 55 of the Point Grey Digital Camera Register Reference Manual
        char bayerBits[5];
        bayerBits[4] = 0;
        uint32_t * bayerInt = reinterpret_cast<uint32_t*>(bayerBits);
        dc1394SafeCall( "Get the sensor Bayer pattern",
	          dc1394_get_control_register,
	          camera_, BAYER_TILE_MAPPING_REGISTER,
	          bayerInt, ASRL_SOURCE_FILE_POS);
        std::string bayerString(bayerBits);
        // TODO: Implement
        printf("Bayer pattern: %s \n",bayerString.c_str());
        bayerPattern_ = stringToPattern(bayerString);
    }

    dc1394SafeCall( "Get the camera serial number",
          dc1394_get_control_register,
          camera_, SERIAL_NUMBER_REGISTER,
          &serialNumber_, ASRL_SOURCE_FILE_POS);

    // TODO: Replace with directory
    if(calibrationFile_.size() == 0) {
        calibrationFile_ = "/tmp/bbxb3-" + std::to_string(serialNumber_) + ".cal";
    }
    printf("Found camera with serial number %d\n",serialNumber_);
   // TODO: What is this?
   // updateFeatures();
    initialized_ = true;
} // end init

///////////////////////////////////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////////////////////////////////
void Camera1394::start(void)
{
    printf("Starting 1394 camera!");
    assert(initialized_); //"The camera is not initialized. Call init() before start()");
    assert(!isCapturing_); // "The camera is already capturing");

    // Configuration file setup...
    // TODO: add config file option.
    // TODO: Is this necessary???
   saveCalibrationFile(calibrationFile_);

    // TODO: What do we need out of the processor?
    // ROS_INFO("Initializing processor");
    // CameraDevice::initProcessor(config_.captureMode_,outputMode_, pointGreyCalibrationFile_);

    printf("Performing safe calls!\n");
    // When the a client program crashes it leaves the bus in
    // a bad state. Resetting the bus allows us to recover the
    // camera in this case. Are there any bad side effects of
    // doing this here?
    dc1394_reset_bus(camera_);

    /*dc1394SafeCall("Reset the camera bus",
       dc1394_reset_bus,camera_,
       ASRL_SOURCE_FILE_POS);*/
    dc1394_memory_load(camera_,0);
    /*dc1394SafeCall("Restore camera default settings",
       dc1394_memory_load,
       camera_,0,
       ASRL_SOURCE_FILE_POS);*/

    // The pan register which camera pair we are capturing from---wide or narrow
    //std::cout << "Setting pan register: 0x" << std::hex << getPanRegisterSetting(config_.captureMode_) << std::dec << std::endl;
    dc1394_set_control_register(camera_,PAN_REGISTER,getPanRegisterSetting(config_.captureMode_));
    /* dc1394SafeCall("Set capture pan register",
       dc1394_set_control_register,
       camera_,
       PAN_REGISTER,
       getPanRegisterSetting(config_.captureMode_),
       ASRL_SOURCE_FILE_POS); */


    printf("Setting Operation Mode \n");
    // Two modes exist.
    //DC1394_OPERATION_MODE_LEGACY = 480,
    //DC1394_OPERATION_MODE_1394B

    // if we are using a higher bus speed, try and set B operation. If it fails
    // revert to legacy mode. If we aren't using 800 speed, just set the legacy mode
    if(config_.busSpeed_ == BUS_800)
    {
        try {
          // Try the higher speed bus.
          dc1394_video_set_operation_mode( camera_, DC1394_OPERATION_MODE_1394B );
        } catch(...) {
            std::cerr << "Setting 1394B mode failed. Trying legacy mode\n";
            // That failed... default to the legacy bus.
            dc1394_video_set_operation_mode(camera_, DC1394_OPERATION_MODE_LEGACY);
            /* dc1394SafeCall("Setting the bus operation mode to LEGACY",
                 dc1394_video_set_operation_mode, camera_, DC1394_OPERATION_MODE_LEGACY,
                 ASRL_SOURCE_FILE_POS); */
        }
    } else
    {
        std::cout << "Setting legacy mode...." << std::endl;
        // just set legacy mode
        dc1394_video_set_operation_mode(camera_, DC1394_OPERATION_MODE_LEGACY);
    }
    // DC1394_ISO_SPEED_100,
    // DC1394_ISO_SPEED_200,
    // DC1394_ISO_SPEED_400,
    // DC1394_ISO_SPEED_800,
    std::cout << "Setting the bus speed to " << fromBusSpeed(config_.busSpeed_) << std::endl;
    dc1394_video_set_iso_speed(camera_, toSpeedType(config_.busSpeed_));
    std::cout << "Finished setting the bus speed" << std::endl;
    /* dc1394SafeCall("Setting the bus speed",
		     dc1394_video_set_iso_speed,
		     camera_, toSpeedType(config_.busSpeed_),
		     ASRL_SOURCE_FILE_POS);*/

    if(config_.captureMode_ == STEREO_WIDE ||
       config_.captureMode_ == STEREO_NARROW ||
       config_.captureMode_ == STEREO_3CAM)
    {
        dc1394_set_control_register(camera_,
        ENDIANNESS_REGISTER,
			  FEATURE_LITTLE_ENDIAN);
        dc1394_video_set_mode(camera_, DC1394_VIDEO_MODE_FORMAT7_3);

        // determine the correct raw resolution. For the XB3 this is 1280*960
        // for the BB2 this is 640*480
        int raw_image_height = DEFAULT_RAW_IMAGE_HEIGHT;
        int raw_image_width = DEFAULT_RAW_IMAGE_WIDTH;
        if(config_.raw_resolution_ == RES_640x480) {
          raw_image_height = 480;
          raw_image_width = 640;
        }

        dc1394_format7_set_roi(
			  camera_,
			  DC1394_VIDEO_MODE_FORMAT7_3,
			  getColorCoding(config_.captureMode_,isMonochrome_),
			  config_.packetSize_, //int32_t packet_size
			  0,                    // ROI left
			  0,                    // ROI top,
			  raw_image_width,      // width
			  raw_image_height);    // height
		    std::cout << "Initializing camera with capture mode " << captureModeToString(config_.captureMode_)
			  << ". The camera is " << (isMonochrome_ ? "" : "not") << " monochrome. The color coding is " <<
			  getColorCoding(config_.captureMode_, isMonochrome_) << std::endl;

/*
        // This makes sure the stereo images come the right way round (I think).
        dc1394SafeCall("Setting up capture endianness",
           dc1394_set_control_register,
           camera_,
           ENDIANNESS_REGISTER,
           FEATURE_LITTLE_ENDIAN,
           ASRL_SOURCE_FILE_POS);
        dc1394SafeCall("Setting Format 7 Capture Mode On",
           dc1394_video_set_mode,
           camera_,
           DC1394_VIDEO_MODE_FORMAT7_3,
           ASRL_SOURCE_FILE_POS);


        // From dcam1394/format7.h
        dc1394SafeCall("Set up Format 7 capture mode",
           dc1394_format7_set_roi,
           camera_,
           DC1394_VIDEO_MODE_FORMAT7_3,
           getColorCoding(config_.captureMode_,isMonochrome_),
           DC1394_USE_MAX_AVAIL, //int32_t packet_size
           0,                    // ROI left
           0,                    // ROI top,
           RAW_IMAGE_WIDTH,  //  width
           RAW_IMAGE_HEIGHT, //  height
           ASRL_SOURCE_FILE_POS);*/
    }

    // "Setup the capture, using a ring buffer of a certain size (num_dma_buffers) and certain options (flags)"
    std::cout << "Setting up " << config_.numDmaBuffers_ << " capture buffers";
    dc1394_capture_setup(camera_, config_.numDmaBuffers_, DC1394_CAPTURE_FLAGS_DEFAULT);
/*
    dc1394SafeCall("Setup the capture buffers",
        dc1394_capture_setup,
        camera_,
        config_.numDmaBuffers_,               // The number of DMA
        DC1394_CAPTURE_FLAGS_DEFAULT, // flags
        ASRL_SOURCE_FILE_POS);
*/

    /*dc1394SafeCall("Starting image transmission",
        dc1394_video_set_transmission,
        camera_, DC1394_ON,
        ASRL_SOURCE_FILE_POS);*/
    dc1394error_t err;
    dc1394switch_t transmissionStatus = DC1394_OFF;

    for(int i = 0; i < config_.transmissionStatusRetries_; i++)
	  {
        err=dc1394_video_set_transmission(camera_, DC1394_ON);
        printf("Starting video transmission err: %i \n",err);
	      usleep(50000);
        dc1394_video_get_transmission(camera_,&transmissionStatus);
/*
	      dc1394SafeCall("Checking for camera wake up",
			      dc1394_video_get_transmission,
			      camera_,
            &transmissionStatus,
            ASRL_SOURCE_FILE_POS);*/
        //int err = dc1394_video_get_transmission( camera_, &transmissionStatus );
        printf("get_transmission status: %i \n",transmissionStatus);
        if(transmissionStatus == DC1394_ON)
        {
            break;
        }
	  }

    assert(transmissionStatus!= DC1394_OFF); // "Unable to wake up camera");
    // unsigned long ulValue;
    // unsigned long ulRegister = 0x12f8;
    // flycaptureSafeCall("Checking if timestamping is enabled",flycaptureGetCameraRegister, context(), ulRegister, &ulValue, ASRL_SOURCE_FILE_POS );

    // ////
    // //// ensure time stamping is present...
    // ////
    // ASRL_ASSERT( ulValue & 0x80000000,"The timestamp feature is not supported---upgrade the firmware");
    // flycaptureSafeCall("Setting timestamping on", flycaptureSetCameraRegister, context(), ulRegister, ulValue | 1, ASRL_SOURCE_FILE_POS );

    // bool isTimestampingOn = false;
    // flycaptureSafeCall("Checking if accurate timestamps are on",flycaptureGetImageTimestamping, context(), &isTimestampingOn, ASRL_SOURCE_FILE_POS);
    // ASRL_ASSERT(isTimestampingOn,"Timestamping is not on!");

    // TODO: Srsly tho
    // updateFeatures();
    isCapturing_ = true;
}

std::shared_ptr<dc1394video_frame_t> Camera1394::getNextFrame()
    {
      // Dequeue the image
      dc1394video_frame_t * rawFrame;
      std::shared_ptr<dc1394video_frame_t> frame;
      // TODO: Figure out if this is the right thing to do...
      // If this wait stalls forever, the node will become unresponsive.
      // I suppose the alternative is to poll or spawn a capture thread

      // TODO: Read to the end of the queue?
      // see the definition of dc1394video_frame_t at /usr/include/dc1394/video.h
      // rawFrame.frames_behind

      // TODO: Does the ID tell me when I've missed frames?
      dc1394_capture_dequeue(
		     camera_,
		     DC1394_CAPTURE_POLICY_WAIT,
		     &rawFrame);
      // dc1394_capture_enqueue(camera_,rawFrame);
      frame.reset(rawFrame,std::bind(&dc1394_capture_enqueue,camera_,std::placeholders::_1));
      return frame;
    }

BayerPattern Camera1394::stringToPattern(std::string const & patternString) {
    return BGGR;
}


uint32_t getPanRegisterSetting(CaptureMode captureMode) {
    uint32_t panRegister = 0x82000000;
    switch(captureMode)
	{
	case LEFT:
	  panRegister = 0x82000002;
	  break;
	case MIDDLE:
	  panRegister = 0x82000001;
	  break;
	case RIGHT:
	  panRegister = 0x82000000;
	  break;
	case STEREO_WIDE:
	  panRegister = 0x82000000;
	  break;
	case STEREO_NARROW:
	  panRegister = 0x82000001;
	  break;
	case STEREO_3CAM:
	  panRegister = 0x82000000;
	  break;
	default:
      break;
	  // ASRL_THROW(Exception, "The camera mode is invalid: " << captureMode);
	}

      return panRegister;
}

dc1394color_coding_t Camera1394::getColorCoding(CaptureMode captureMode, bool cameraIsMonochrome) {
    dc1394color_coding_t colorCoding =  DC1394_COLOR_CODING_MONO8;
    switch(captureMode)
	{
	case LEFT:
	case MIDDLE:
	case RIGHT:
	  colorCoding = DC1394_COLOR_CODING_MONO8;
	  break;
	case STEREO_WIDE:
	case STEREO_NARROW:
	  // DC1394_COLOR_CODING_MONO16 doesn't work for the BBXB3 Color cameras.
	  // I don't have a mono camera so I'm not sure if this will be a problem.
	  if(cameraIsMonochrome) {
	    colorCoding = DC1394_COLOR_CODING_MONO16;
	  }
	  else {
	    colorCoding = DC1394_COLOR_CODING_RAW16;
	  }
	  break;
	case STEREO_3CAM:
	  colorCoding = DC1394_COLOR_CODING_RGB8;
	  break;
	default:
        break;
	};
    return colorCoding;
}


std::string Camera1394::captureModeToString(CaptureMode captureMode) {
    return captureModeNames[captureMode];
}

 BusSpeed toBusSpeed(std::string const & busSpeedString)
    {
      BusSpeed busSpeed = BUS_100;
      if(busSpeedString == "100")
	{
	  busSpeed = BUS_100;
	}
      else if(busSpeedString == "200")
	{
	  busSpeed = BUS_200;
	}
      else if(busSpeedString == "400")
	{
	  busSpeed = BUS_400;
	}
      else if(busSpeedString == "800")
	{
	  busSpeed = BUS_800;
	}
      return busSpeed;
}


std::string fromBusSpeed(BusSpeed busSpeed) {
      std::string busSpeedString;
      switch(busSpeed)
	{
	case BUS_100:
	  busSpeedString = "100";
	  break;
	case BUS_200:
	  busSpeedString = "200";
	  break;
	case BUS_400:
	  busSpeedString = "400";
	  break;
	case BUS_800:
	  busSpeedString = "800";
	  break;
	default:
	  break;
	};

  return busSpeedString;
}

dc1394speed_t toSpeedType(BusSpeed busSpeed) {
    dc1394speed_t speed = DC1394_ISO_SPEED_800;
  switch(busSpeed)
	{
	case BUS_100:
	  speed = DC1394_ISO_SPEED_100;
	  break;
	case BUS_200:
	  speed = DC1394_ISO_SPEED_200;
	  break;
	case BUS_400:
	  speed = DC1394_ISO_SPEED_400;
	  break;
	case BUS_800:
	  speed = DC1394_ISO_SPEED_800;
	  break;
	default:
      break;
	}

      return speed;
}
