/*
 * Unpublished Work Copyright 2019 Waymo LLC.  All rights reserved.
 * Waymo Proprietary and Confidential - Contains Trade Secrets
 *
 * This is the proprietary software of Waymo LLC ("Waymo") and/or its licensors,
 * and may only be used, duplicated, modified or distributed pursuant to the
 * terms and conditions of a separate, written license agreement executed
 * between you and Waymo (an "Authorized License"). Except as set forth in an
 * Authorized License, Waymo grants no license (express or implied), right to
 * use, or waiver of any kind with respect to the Software, and Waymo expressly
 * reserves all rights in and to the Software and all intellectual property
 * rights therein. IF YOU HAVE NO AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO
 * USE THIS SOFTWARE IN ANY WAY, AND SHOULD IMMEDIATELY NOTIFY WAYMO AND
 * DISCONTINUE ALL USE OF THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License:
 *
 * 1. This software includes trade secrets of Waymo, and you shall use all
 * reasonable efforts to protect the confidentiality thereof.  You shall use
 * this software only in connection with your authorized use of Waymo products.
 *
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, WAYMO PROVIDES THE
 * WAYMO SOFTWARE ON AN “AS IS” AND “AS AVAILABLE” BASIS WITH ALL FAULTS AND
 * WITHOUT ANY REPRESENTATIONS OR WARRANTIES OF ANY KIND.  WAYMO EXPRESSLY
 * DISCLAIMS ALL WARRANTIES, WHETHER IMPLIED, STATUTORY OR OTHERWISE, INCLUDING
 * IMPLIED WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT AND FITNESS FOR
 * PARTICULAR PURPOSE OR ARISING FROM COURSE OF PERFORMANCE, COURSE OF DEALING
 * OR USAGE OF TRADE.  SOME JURISDICTIONS DO NOT ALLOW THE EXCLUSION OF CERTAIN
 * WARRANTIES, REPRESENTATIONS OR CONDITIONS, THE LIMITATION OR EXCLUSION OF
 * IMPLIED WARRANTIES, LIMITATIONS ON HOW LONG AN IMPLIED WARRANTY MAY LAST OR
 * EXCLUSIONS OR LIMITATIONS FOR CONSEQUENTIAL OR INCIDENTAL DAMAGES, SO SOME OF
 * THE ABOVE LIMITATIONS MAY NOT APPLY IN FULL TO YOU, AND WAYMO’S LIABILITY
 * SHALL BE LIMITED TO THE EXTENT SUCH LIMITATIONS ARE PERMITTED BY LAW.
 *
 * 3. TO THE MAXIMUM EXTENT PERMITTED UNDER APPLICABLE LAW, IN NO EVENT WILL
 * WAYMO OR ITS LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL,
 * INCIDENTAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES OF ANY KIND ARISING
 * OUT OF OR IN CONNECTION WITH THE WAYMO SOFTWARE, REGARDLESS OF THE FORM OF
 * ACTION, WHETHER IN CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT LIABILITY OR
 * OTHERWISE, EVEN IF WAYMO HAS BEEN ADVISED OR IS OTHERWISE AWARE OF THE
 * POSSIBILITY OF SUCH DAMAGES. THE FOREGOING LIMITATIONS, EXCLUSIONS, AND
 * DISCLAIMERS WILL APPLY TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW,
 * EVEN IF ANY REMEDY FAILS ITS ESSENTIAL PURPOSE.
 */
#ifndef HONEYCOMB_C_API_H_
#define HONEYCOMB_C_API_H_

#include <stddef.h>
#include <stdint.h>


#define WL_CAPI_EXPORT __attribute__((visibility("default")))

#ifdef __has_attribute
#define WL_HAS_ATTRIBUTE(x) __has_attribute(x)
#else
#define WL_HAS_ATTRIBUTE(x) 0
#endif

#if WL_HAS_ATTRIBUTE(nodiscard)
#define WL_MUST_USE_RESULT [[nodiscard]]
#elif defined(__clang__) && WL_HAS_ATTRIBUTE(warn_unused_result)
#define WL_MUST_USE_RESULT __attribute__((warn_unused_result))
#else
#define WL_MUST_USE_RESULT
#endif

#ifdef __cplusplus
#define WL_ENUM_CLASS enum class
namespace waymo {
#else
#define WL_ENUM_CLASS enum
#include <stdbool.h>
#endif

typedef WL_ENUM_CLASS WL_MUST_USE_RESULT WL_Status{
    kOk = 0,
    kErrorInternal,
    kErrorStopped,
    kErrorInvalidArgument,
    kErrorNoDevice,
    kErrorNoMatchingDevice,
    kErrorTooManyDevices,
    kErrorTimeout,
    kErrorUninitialized,
} WL_Status;

// The severity levels indicate the seriousness of the error reported.
// Higher values are used for more severe errors.
typedef WL_ENUM_CLASS WL_Severity{
    kDebug,
    kLog,
    kWarning,
    kError,
} WL_Severity;

// An enum to indicate if an operation should occur on the front side, back
// side, both sides, or neither side of the Honeycomb lidar.
typedef WL_ENUM_CLASS WL_Sides{
    kNone = 0,
    kFrontSide = 1,
    kBackSide = 2,
    kBothSides = 3,
} WL_Sides;

// The natural data type of the accompanying statistical value.
typedef WL_ENUM_CLASS WL_StatType{
    kBool,
    kInteger,
    kFloat,
} WL_StatType;

typedef WL_ENUM_CLASS WL_SystemStatus{
    kUninitialized = 0, kInitializing, kInitialized, kRunning, kShutdown,
} WL_SystemStatus;

typedef struct WL_Calibration {
  // Distance offset, in meters, added to the range for each return.
  double distance_offset;

  // The angle, in degrees, used to align the pitch between the two sides of the
  // sensor.
  double elevation_adjustment;

  // The angle, in degrees, used to align the yaw between the two sides of the
  // sensor.
  double azimuth_adjustment;

  // The threshold for the comparator above which we trigger a return.  This
  // value is in the range [0, 255].
  double comparator_threshold;
} WL_Calibration;

typedef struct WL_ImuVector {
  float x;
  float y;
  float z;
} WL_ImuVector;

typedef struct WL_ImuSample {
  // Unix epoch time in seconds for this sample.
  double timestamp;

  // The angular velocity vector in deg/s
  WL_ImuVector angular_velocity;

  // The linear acceleration vector in G's
  // An IMU that is placed in its neutral right-side-up position on a flat
  // surface will:
  //   - Measure +9.81 meters per second squared for the Z axis.
  //   - If the sensor is rolled +90 degrees (left side up), the
  //     acceleration should be +9.81 meters per second squared for the
  //     Y axis.
  //   - If the  sensor is pitched +90 degrees (front side down), it
  //     should read -9.81 meters per second squared for the X axis.
  WL_ImuVector linear_acceleration;
} WL_ImuSample;

// This struct describes the limits of the programmable pitch table for the
// current device, as returned by GetPitchTableLimits.
typedef struct WL_PitchTableLimits {
  // The minimum supported pitch angle, in degrees.
  double min_degrees;

  // The maximum supported pitch angle, in degrees.
  double max_degrees;

  // The minimum difference between adjacent entries in the pitch table, in
  // degrees.
  double min_spacing_degrees;

  // The maximum number of pitch table entries.
  int max_entries;
} WL_PitchTableLimits;

typedef struct WL_SystemInfo {
  char* ip_address;
  char* mac_address;
  char* model_number;
  char* serial_number;
  char* software_build_version;
  char* software_build_date;
} WL_SystemInfo;

typedef struct WL_SystemError {
  char* code;
  char* description;
  double timestamp;
  WL_Severity severity;
} WL_SystemError;

const double WL_MinFieldOfView = 1.0;
const double WL_MaxFieldOfView = 360.0;
const double WL_MinDirection = -180.0;
const double WL_MaxDirection = 180.0;
const double WL_MinSpinFrequency = 5.0;
const double WL_MaxSpinFrequency = 15.0;
const double WL_MinVerticalScanFrequency = 1477.5;
const double WL_MaxVerticalScanFrequency = 1500.0;
const double WL_MinPitchAngle = -78;
const double WL_MaxPitchAngle = 20.75;
const double WL_MinPitchDifferential = 0.75;
const int WL_MinScansPerShard = 0;
const int WL_MaxScansPerShard = 1000;

#ifdef __cplusplus
extern "C" {
namespace internal {
#endif

typedef struct WL_Scans WL_Scans;
typedef struct WL_LidarHandle WL_LidarHandle;
typedef struct WL_PitchTable WL_PitchTable;

// Initialization and control
WL_CAPI_EXPORT WL_Status WL_ScanForDevices(int* num_devices, char*** hostnames,
                                           char*** mac_addresses);
WL_CAPI_EXPORT WL_Status WL_NewLidarHandle(const char* hostname,
                                           const char* mac_address,
                                           WL_LidarHandle** lidar);
WL_CAPI_EXPORT WL_Status WL_DeleteLidarHandle(WL_LidarHandle* lidar);
WL_CAPI_EXPORT WL_Status WL_InitLidar(WL_LidarHandle* lidar);
WL_CAPI_EXPORT WL_Status WL_RunLidar(WL_LidarHandle* lidar);
WL_CAPI_EXPORT WL_Status WL_StopLidar(WL_LidarHandle* lidar);
WL_CAPI_EXPORT WL_Status WL_IsRunning(WL_LidarHandle* lidar, bool* running);
WL_CAPI_EXPORT WL_Status WL_IsSteadyState(WL_LidarHandle* lidar, bool* steady);

// Retrieving shot data
WL_CAPI_EXPORT WL_Status WL_GetLidarData(WL_LidarHandle* lidar, double timeout,
                                         bool latest, WL_Scans** scans);
WL_CAPI_EXPORT WL_Status WL_ReleaseLidarData(WL_LidarHandle* lidar,
                                             WL_Scans* scans);

// Retrieving IMU data
WL_CAPI_EXPORT WL_Status WL_GetImuData(WL_LidarHandle* lidar, double timeout,
                                       WL_ImuSample* sample_array[],
                                       int* num_samples);

// Retrieving system stats
WL_CAPI_EXPORT WL_Status WL_GetStatsData(WL_LidarHandle* lidar, double timeout,
                                         bool latest, int* num_stats,
                                         char*** stat_keys,
                                         WL_StatType** stat_types,
                                         char*** stat_values);

// Get/Set laser configuration
WL_CAPI_EXPORT WL_Status WL_SetSpinFrequency(WL_LidarHandle* lidar,
                                             double frequency);
WL_CAPI_EXPORT WL_Status WL_GetSpinFrequency(WL_LidarHandle* lidar,
                                             double* frequency);

WL_CAPI_EXPORT WL_Status WL_SetVerticalScanFrequency(WL_LidarHandle* lidar,
                                                     double frequency);
WL_CAPI_EXPORT WL_Status WL_GetVerticalScanFrequency(WL_LidarHandle* lidar,
                                                     double* frequency);

WL_CAPI_EXPORT WL_Status WL_SetFieldOfView(WL_LidarHandle* lidar, double fov,
                                           double direction);
WL_CAPI_EXPORT WL_Status WL_GetFieldOfView(WL_LidarHandle* lidar, double* fov,
                                           double* direction);

WL_CAPI_EXPORT WL_Status WL_SetSides(WL_LidarHandle* lidar, WL_Sides sides);
WL_CAPI_EXPORT WL_Status WL_GetSides(WL_LidarHandle* lidar, WL_Sides* sides);

WL_CAPI_EXPORT WL_Status WL_SetComputeNormals(WL_LidarHandle* lidar,
                                              WL_Sides sides);
WL_CAPI_EXPORT WL_Status WL_GetComputeNormals(WL_LidarHandle* lidar,
                                              WL_Sides* sides);

WL_CAPI_EXPORT WL_Status WL_SetPitchTable(WL_LidarHandle* lidar,
                                          const double pitches[],
                                          int num_pitches, bool interlaced);
WL_CAPI_EXPORT WL_Status WL_GetPitchTable(WL_LidarHandle* lidar,
                                          double* pitches[], int* num_pitches,
                                          bool* interlaced);
WL_CAPI_EXPORT WL_Status WL_GetPitchTableLimits(WL_LidarHandle* lidar,
                                                WL_PitchTableLimits* limits);

WL_CAPI_EXPORT WL_Status WL_SetScansPerShard(WL_LidarHandle* lidar,
                                             int num_scans);
WL_CAPI_EXPORT WL_Status WL_GetScansPerShard(WL_LidarHandle* lidar,
                                             int* num_scans);

WL_CAPI_EXPORT WL_Status WL_SetDropNoiseReturns(WL_LidarHandle* lidar,
                                                bool enabled);
WL_CAPI_EXPORT WL_Status WL_GetDropNoiseReturns(WL_LidarHandle* lidar,
                                                bool* enabled);


WL_CAPI_EXPORT WL_Status WL_GetSystemErrors(WL_LidarHandle* lidar,
                                            WL_SystemError* system_errors[],
                                            int* num_errors);
WL_CAPI_EXPORT WL_Status WL_GetSystemInfo(WL_LidarHandle* lidar,
                                          WL_SystemInfo* system_info);
WL_CAPI_EXPORT WL_Status WL_GetSystemStatus(WL_LidarHandle* lidar,
                                            WL_SystemStatus* system_status);

// Open source license notices
WL_CAPI_EXPORT const char* WL_GetLicenseNotices();

#ifdef __cplusplus
namespace experimental {
#endif
WL_CAPI_EXPORT WL_Status WL_GetCalibrationParameters(WL_LidarHandle* lidar,
                                                     bool persist,
                                                     WL_Calibration* cal);
WL_CAPI_EXPORT WL_Status WL_SetCalibrationParameters(WL_LidarHandle* lidar,
                                                     bool persist,
                                                     WL_Calibration* cal);
#ifdef __cplusplus
}  // namespace experimental

}  // namespace internal
}  // extern "C"
}  // namespace waymo
#endif

#endif  // HONEYCOMB_C_API_H_
