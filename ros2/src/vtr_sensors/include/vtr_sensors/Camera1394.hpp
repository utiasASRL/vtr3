#pragma once

#include <dc1394/control.h>
#include <array>
#include <memory>

const std::string XB3_MODEL_NAME = std::string("Bumblebee XB3");
const std::string BB2_MODEL_NAME = std::string("Bumblebee2 BB2-03S2C");

enum Register {
  // Page 78 of the Point Grey Digital Camera Register Reference Manual
  SENSOR_INFO_REGISTER = 0x1f28,
  // Page 55 of the Point Grey Digital Camera Register Reference Manual
  BAYER_TILE_MAPPING_REGISTER = 0x1040,
  // Page 42 of the Point Grey Digital Camera Register Reference Manual
  PAN_REGISTER = 0x884,
  // Page 56 of the Point Grey Digital Camera Register Reference Manual
  ENDIANNESS_REGISTER = 0x1048,
  // Page 78 of the Point Grey Digital Camera Register Reference Manual
  SERIAL_NUMBER_REGISTER = 0x1F20,
  CONFIG_LENGTH_REGISTER = 0x1FFC,
  CONFIG_DATA_REGISTER = 0x2000,
  ACCURATE_TIMESTAMPS_REGISTER = 0x12f8
};

// The leading 8 means "This feature is available"
enum Endianness {
  FEATURE_BIG_ENDIAN = 0x80000001,
  FEATURE_LITTLE_ENDIAN = 0x80000000
};

const int DEFAULT_RAW_IMAGE_HEIGHT = 960;
const int DEFAULT_RAW_IMAGE_WIDTH = 1280;
enum BusSpeed { BUS_100, BUS_200, BUS_400, BUS_800 };

BusSpeed toBusSpeed(std::string const &busSpeedString);
std::string fromBusSpeed(BusSpeed busSpeed);
dc1394speed_t toSpeedType(BusSpeed busSpeed);

enum Resolution {
  RES_1280x960 = 0,
  RES_1024x768,
  RES_640x480,
  RES_512x384,
  RES_320x240,
  N_RESOLUTIONS
};

enum OutputMode {
  DE_INTERLEAVED = 0,  // The raw image, with the pixels de-interleaved
  MONO_FAST_DOWNSAMPLE =
      1,  // The image converted to monochrome by grabbing green pixels
  //     The resulting image will be half the size of the original
  //     640x480
  N_OUTPUT_MODES = 2
};

enum CaptureMode {
  LEFT = 0,
  MIDDLE = 1,
  RIGHT = 2,
  STEREO_WIDE = 3,
  STEREO_NARROW = 4,
  STEREO_3CAM = 5,
  N_CAPTURE_MODES = 6
};

enum BayerPattern { BGGR = 0, GBRG, RGGB, GRBG, YYYY, N_PATTERNS };
static std::array<std::string, 6> captureModeNames = {
    "left", "middle", "right", "stereo_wide", "stereo_narrow", "stereo_3cam"};

uint32_t getPanRegisterSetting(CaptureMode captureMode);

/// @brief Implementation of lower level XB3 functions
class Camera1394 {
 public:
  struct Config {
    /// Configuration:
    CaptureMode captureMode_;

    /// Number of buffers that the driver manages
    int numDmaBuffers_;

    /// Desired Bus Speed
    BusSpeed busSpeed_;

    /// Number of times to poll the camera for a successful startup.
    int transmissionStatusRetries_;

    Resolution raw_resolution_;

    uint32_t packetSize_;
  };

  Camera1394();
  explicit Camera1394(Config config);
  ~Camera1394();

  void init();
  void stop();
  void start();

  void saveCalibrationFile(const std::string &file);

  std::shared_ptr<dc1394video_frame_t> getNextFrame();
  std::string calibrationFile() { return calibrationFile_; };

 private:
  static dc1394color_coding_t getColorCoding(CaptureMode captureMode,
                                      bool isCameraMonochrome);

  static std::string captureModeToString(CaptureMode captureMode);

  void cleanUp();

  /// The firewire camera struct.
  dc1394camera_t *camera_;

  /// Has the camera been initialized.
  bool initialized_;

  /// Is the camera capturing?
  bool isCapturing_;

  /// Is the camera monochrome?
  bool isMonochrome_;

  /// The camera serial number
  uint32_t serialNumber_;

  /// Path to the configuration file
  std::string calibrationFile_;

  Config config_;
};
