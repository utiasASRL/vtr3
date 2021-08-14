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
#ifndef HONEYCOMB_H_
#define HONEYCOMB_H_

#include <endian.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "honeycomb/honeycomb_c_api.h"
#include "honeycomb/honeycomb_scans.h"

#if (defined(__GNUC__) || defined(__clang__)) && __cplusplus >= 201103L
#define WL_DEPRECATED(message) __attribute__((deprecated(message)))
#endif

#ifndef WL_DEPRECATED
#define WL_DEPRECATED(message)
#endif

namespace waymo {

using Calibration = WL_Calibration;
using ImuSample = WL_ImuSample;
using ImuVector = WL_ImuVector;
using PitchTableLimits = WL_PitchTableLimits;
using Severity = WL_Severity;
using Sides = WL_Sides;
using StatType = WL_StatType;
using Status = WL_Status;
using SystemStatus = WL_SystemStatus;

constexpr double kDurationForever = std::numeric_limits<double>::infinity();
constexpr bool kIsBigEndianHost = (__BYTE_ORDER == __BIG_ENDIAN);

inline double Rad2Deg(double rad) { return rad * 180.0 / M_PI; }
inline float Rad2Deg(float rad) { return rad * 180.0 / M_PI; }

inline const std::string SystemStatusToString(SystemStatus system_status) {
  switch (system_status) {
    case SystemStatus::kUninitialized:
      return "UNINITIALIZED";
    case SystemStatus::kInitializing:
      return "INITIALIZING";
    case SystemStatus::kInitialized:
      return "INITIALIZED";
    case SystemStatus::kRunning:
      return "RUNNING";
    case SystemStatus::kShutdown:
      return "SHUTDOWN";
    default:
      return "UNKNOWN";
  }
}
inline std::ostream& operator<<(std::ostream& out, SystemStatus system_status) {
  return out << SystemStatusToString(system_status);
}

inline const std::string StatusToString(Status status) {
  switch (status) {
    case Status::kOk:
      return "OK";
    case Status::kErrorInternal:
      return "ERROR_INTERNAL";
    case Status::kErrorStopped:
      return "ERROR_STOPPED";
    case Status::kErrorInvalidArgument:
      return "ERROR_INVALID_ARGUMENT";
    case Status::kErrorNoDevice:
      return "ERROR_NO_DEVICE";
    case Status::kErrorNoMatchingDevice:
      return "ERROR_NO_MATCHING_DEVICE";
    case Status::kErrorTooManyDevices:
      return "ERROR_TOO_MANY_DEVICES";
    case Status::kErrorTimeout:
      return "ERROR_TIMEOUT";
    case Status::kErrorUninitialized:
      return "ERROR_UNINITIALIZED";
    default:
      return "UNKNOWN";
  }
}
inline std::ostream& operator<<(std::ostream& out, Status status) {
  out << StatusToString(status);
  return out;
}

inline const std::string SeverityToString(Severity severity) {
  switch (severity) {
    case Severity::kDebug:
      return "DEBUG";
    case Severity::kLog:
      return "LOG";
    case Severity::kWarning:
      return "WARNING";
    case Severity::kError:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}
inline std::ostream& operator<<(std::ostream& out, Severity severity) {
  out << SeverityToString(severity);
  return out;
}

inline const std::string SidesToString(Sides sides) {
  switch (sides) {
    case Sides::kNone:
      return "NONE";
    case Sides::kFrontSide:
      return "FRONT_SIDE";
    case Sides::kBackSide:
      return "BACK_SIDE";
    case Sides::kBothSides:
      return "BOTH_SIDES";
    default:
      return "UNKNOWN";
  }
}
inline std::ostream& operator<<(std::ostream& out, Sides sides) {
  out << SidesToString(sides);
  return out;
}

/// This class is the starting point for all operations of the Honeycomb lidar
/// device. Once you've initialized it with the device address and configured
/// its options, you can use it to read point cloud data from the device.
///
/// An example of its use (error checking removed for clarity):
///
///     // Create Honeycomb and initialize it
///     waymo::Honeycomb lidar();
///     lidar.Init(hostname, mac_address);
///
///     // Optionally, configure lidar parameters, as necessary
///     lidar.SetSpinFrequency(10.0);  // in Hz
///     lidar.SetFieldOfView(210, 0);  // in degrees
///     ...
///     lidar.SetPitchTable(pitch_table, interlaced_mode);  // vector of angles
///
///     // Start collecting data
///     lidar.Run();
///
///     // Process point cloud data
///     while (program_is_ok && lidar.IsRunning()) {
///       waymo::ScansPtr scans;
///       lidar.GetLidarData(waymo::kDurationForever, True, &scans);
///       DoSomethingWithData(scans.get());
///     }
///
///     lidar.Stop();
class Honeycomb {
 public:
  Honeycomb() = default;

  /// @brief The destructor cleanly shuts down the Honeycomb device and releases
  /// its resources.
  virtual ~Honeycomb() {
    if (hc_) {
      Status status __attribute__((unused)) =
          internal::WL_DeleteLidarHandle(hc_);
    }
  }

  struct DeviceAddress {
    std::string ip_address;
    std::string mac_address;
  };

  /// @brief Scan the available network interfaces for Honeycomb lidar devices.
  ///
  /// This method will scan all available network interfaces, returning a vector
  /// of IP address/MAC address pairs for each Honeycomb lidar device it
  /// discovers.
  ///
  /// @param[out] devices A pointer to a vector of device addresses.
  ///
  /// @return Status::kOk if the scan completed successfully, an error
  /// otherwise.
  virtual Status ScanForDevices(std::vector<DeviceAddress>* devices) {
    int num_devices;
    char** hostnames;
    char** mac_addresses;
    Status status;
    if ((status = internal::WL_ScanForDevices(&num_devices, &hostnames,
                                              &mac_addresses)) == Status::kOk) {
      for (int i = 0; i < num_devices; ++i) {
        DeviceAddress a = {hostnames[i], mac_addresses[i]};
        devices->push_back(a);
        free(static_cast<void*>(hostnames[i]));
        free(static_cast<void*>(mac_addresses[i]));
      }
      free(hostnames);
      free(mac_addresses);
    }
    return status;
  }

  /// @brief Initializes the Honeycomb lidar device at the specified address.
  ///
  /// Establishes a network connection to lidar device, initializes it, performs
  /// a self-test, and starts the motor to spin the housing.
  ///
  /// The initialization process can take ~30 seconds.  This call blocks while
  /// waiting for the device to become available.  You may wish to perform the
  /// initialization in a separate thread.
  ///
  /// @param[in] hostname The hostname or IP address to assign the device.  The
  /// device must be plugged into an interface whose subnet matches the IP
  /// address provided. The driver will attempt to scan the network for a
  /// Honeycomb device if the empty string is provided.
  ///
  /// @param[in] mac_address The MAC address of the device.  The MAC address
  /// can be used to differentiate multiple devices on the same subnet.
  /// The driver will attempt to scan the network for a Honeycomb device if
  /// the empty string is provided.
  ///
  /// @return Status::kOk if the device was successfully initialized, an error
  /// otherwise:
  ///   - Status::kNoDevice is returned if no Honeycomb devices can be
  ///     detected.
  ///   - Status::kErrorNoMatchingDevice is returned if a MAC address is
  ///   provided,
  ///     but a matching device can not be detected.
  ///   - Status::kErrorTooManyDevices is returned if no hostname or MAC
  ///   address
  ///     is specified, but more than one Honeycomb device is detected.
  virtual Status Init(std::string hostname, std::string mac_address) {
    Status status = internal::WL_NewLidarHandle(hostname.c_str(),
                                                mac_address.c_str(), &hc_);
    if (status == Status::kOk) {
      status = internal::WL_InitLidar(hc_);
    }
    return status;
  }
  virtual Status Init(std::string hostname) { return Init(hostname, ""); }
  virtual Status Init() { return Init("", ""); }

  /// @brief Starts collecting point cloud data.
  ///
  /// Once the lidar has been initialized, the Run method starts the data
  /// collection.  After Run has been called, point cloud data can be retrieved
  /// with the GetLidarData method.
  ///
  /// @return Status::kOk if the device was successfully started, an error
  /// otherwise.
  virtual Status Run() { return internal::WL_RunLidar(hc_); }

  /// @brief Stops the lidar device.
  ///
  /// Stop the collection of point cloud data and shutdown the motor.  In order
  /// to restart the lidar device, you must call Init followed by Run.
  ///
  /// @return Status::kOk if the device was successfully stopped, an error
  /// otherwise.
  virtual Status Stop() { return internal::WL_StopLidar(hc_); }

  /// @brief Tests if the lidar is running.
  ///
  /// @return True if the lidar is running.
  virtual bool IsRunning() {
    bool running;
    Status status = internal::WL_IsRunning(hc_, &running);
    return status == Status::kOk && running;
  }

  /// @brief Tests if the lidar is in steady state.
  ///
  /// @return True if the lidar is in steady state.
  virtual bool IsSteadyState() {
    bool steady;
    Status status = internal::WL_IsSteadyState(hc_, &steady);
    return status == Status::kOk && steady;
  }


  /// @brief Gets the point cloud data from the lidar device.
  ///
  /// Wait up to `timeout` seconds for a set of point cloud Scans to become
  /// available.  If no data becomes available before the timeout is reached,
  /// then Status::kErrorTimeout is returned. Setting `timeout` to
  /// waymo::kDurationForever will make the call block forever, until data
  /// becomes available.  A `timeout` setting of 0 will make this call
  /// non-blocking.
  ///
  /// The driver queues a limited amount of point data internally. By setting
  /// `latest` to false, this call will return the next available data from the
  /// queue.  Setting `latest` to true will return the most recent point cloud
  /// data, discarding any older data that may have been previously queued.
  ///
  /// On success, this call will return a ScansPtr.
  ///
  /// @param[in] timeout The number of seconds to wait.
  /// @param[in] latest Whether to get the most recent data available.
  /// @param[out] scans A pointer to the point cloud data.
  ///
  /// @return Status::kOk if the data was successfully retrieved, an error
  /// otherwise.
  virtual Status GetLidarData(double timeout, bool latest, ScansPtr* scans) {
    internal::WL_Scans* wl_scans;
    Status status = internal::WL_GetLidarData(hc_, timeout, latest, &wl_scans);
    if (status == Status::kOk) {
      Scans* scans_ptr =
          new Scans(*reinterpret_cast<internal::ScansData*>(wl_scans));
      *scans = ScansPtr(scans_ptr, [this, wl_scans](Scans* p) {
        Status status __attribute__((unused)) =
            internal::WL_ReleaseLidarData(this->hc_, wl_scans);
        delete p;
      });
    }
    return status;
  }

  /// @brief This API is deprecated.  Use GetLidarData instead.
  WL_DEPRECATED("Use GetLidarData() instead.")
  virtual Status GetLaserData(double timeout, bool latest, ScansPtr* scans) {
    return GetLidarData(timeout, latest, scans);
  }

  /// @brief Get IMU data from the lidar device.
  ///
  /// Wait up to `timeout` seconds for a set of IMU samples to become
  /// available.  If no data becomes available before the timeout is reached,
  /// then Status::kErrorTimeout is returned. Setting `timeout` to
  /// waymo::kDurationForever will make the call block until data
  /// becomes available.  A `timeout` setting of 0 will make this call
  /// non-blocking.
  ///
  /// On success, this call will return a std::vector of samples.
  ///
  /// @param[in] timeout The number of seconds to wait.
  /// @param[out] samples A pointer to the vector of IMU samples.
  ///
  /// @return Status::kOk if the data was successfully retrieved, an error
  /// otherwise.
  virtual Status GetImuData(double timeout, std::vector<ImuSample>* samples) {
    int num_samples;
    ImuSample* sample_array;

    if (samples == nullptr) {
      return Status::kErrorInvalidArgument;
    }
    Status status =
        internal::WL_GetImuData(hc_, timeout, &sample_array, &num_samples);
    if (status == Status::kOk) {
      *samples =
          std::vector<ImuSample>(sample_array, sample_array + num_samples);
      free(sample_array);
    }
    return status;
  }

  class StatValue {
   public:
    StatValue(StatType type, std::string value) : type_(type), value_(value) {}

    StatType GetType() const { return type_; }
    std::string GetValue() const { return value_; }
    double GetDoubleValue() const { return std::stod(value_); }
    int64_t GetIntegerValue() const { return std::stoll(value_); }
    bool GetBooleanValue() const { return std::stod(value_) != 0; }

   private:
    StatType type_;
    std::string value_;
  };
  using SystemStats = std::map<std::string, StatValue>;

  /// @brief Gets statistics from the lidar device.
  ///
  /// Wait up to `timeout` seconds for statistics to become
  /// available.  If no data becomes available before the timeout is reached,
  /// then Status::kErrorTimeout is returned. Setting `timeout` to
  /// waymo::kDurationForever will make the call block forever, until data
  /// becomes available.  A `timeout` setting of 0 will make this call
  /// non-blocking.
  ///
  /// The driver queues a limited amount of statistics data internally. By
  /// setting `latest` to false, this call will return the next available data
  /// from the queue.  Setting `latest` to true will return the most recent
  /// statistics data, discarding any older data that may have been previously
  /// queued.
  ///
  /// On success, this call will return a map of stats values.
  ///
  /// @param[in] timeout The number of seconds to wait.
  /// @param[in] latest Whether to get the most recent data available.
  /// @param[out] stats A pointer to a map of stat datatype/value pairs.
  ///
  /// @return Status::kOk if the data was successfully retrieved, an error
  /// otherwise.
  virtual Status GetStatsData(double timeout, bool latest, SystemStats* stats) {
    if (stats == nullptr) {
      return Status::kErrorInvalidArgument;
    }
    int num_stats;
    char** stat_keys;
    StatType* stat_types;
    char** stat_values;
    const auto status =
        internal::WL_GetStatsData(hc_, timeout, latest, &num_stats, &stat_keys,
                                  &stat_types, &stat_values);
    if (status == Status::kOk) {
      for (int i = 0; i < num_stats; ++i) {
        stats->insert(std::make_pair(
            stat_keys[i],
            StatValue(stat_types[i], std::string(stat_values[i]))));
        free(static_cast<void*>(stat_keys[i]));
        free(static_cast<void*>(stat_values[i]));
      }
      free(stat_keys);
      free(stat_types);
      free(stat_values);
    }
    return status;
  }

  /// @brief Sets the horizontal scanning rate.
  ///
  /// Sets the rate at which the housing spins.  Higher frequencies provide
  /// faster data update rates, while lower frequencies provide higher
  /// horizontal resolution.
  ///
  /// @param[in] frequency The frequency in Hz.  This value must be in the range
  /// of [5.0 - 15.0]. Defaults to `10.0`.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetSpinFrequency(double frequency) {
    return internal::WL_SetSpinFrequency(hc_, frequency);
  }

  /// @brief Gets the horizontal scanning rate.
  ///
  /// @param[out] frequency The frequency in Hz.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetSpinFrequency(double* frequency) {
    return internal::WL_GetSpinFrequency(hc_, frequency);
  }

  /// @brief Sets the vertical scanning rate.
  ///
  /// The ratio of vertical scanning rate to horizontal scanning rate controls
  /// interlacing of data in between spins.  A ratio that is an integer will
  /// provide scans in the same place (subject to mechanical jitter) each spin.
  ///
  /// @param[in] frequency The frequency in Hz.  This value must be in the range
  /// of [1477.5 - 1500.0]. Defaults to `1488.54`.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetVerticalScanFrequency(double frequency) {
    return internal::WL_SetVerticalScanFrequency(hc_, frequency);
  }

  /// @brief Gets the vertical scanning rate.
  ///
  /// @param[out] frequency The frequency in Hz.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetVerticalScanFrequency(double* frequency) {
    return internal::WL_GetVerticalScanFrequency(hc_, frequency);
  }

  /// @brief Sets the horizontal field of view.
  ///
  /// Set the field of view to be `fov` degrees wide, centered about the heading
  /// specified by `direction`.  A `direction` of 0 corresponds to straight
  /// ahead.
  ///
  /// @param[in] fov The field of view in degrees. This value must be in the
  /// range of [1, 360]. Defaults to `210`.
  /// @param[in] direction The heading of the field of view. This value must be
  /// in the range of [-180, 180]. Defaults to `0`.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetFieldOfView(double fov, double direction) {
    return internal::WL_SetFieldOfView(hc_, fov, direction);
  }

  /// @brief Gets field of view settings.
  ///
  /// @param[out] fov The field of view in degrees.
  /// @param[out] direction The heading of the field of view in degrees.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetFieldOfView(double* fov, double* direction) {
    return internal::WL_GetFieldOfView(hc_, fov, direction);
  }

  /// @brief Legacy method to operate the device in one-sided mode.
  ///
  /// By default, the lidar will fire shots from the front and back of the
  /// device, effectively doubling the update rate and horizontal resolution. In
  /// one-sided mode, the lidar will only fire shots from the front of the
  /// device.
  ///
  /// @param[in] one_sided If set to true, one-sided mode is enabled. Defaults
  /// to `false`.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  WL_DEPRECATED("Use SetSides() instead.")
  virtual Status SetOneSided(bool one_sided) {
    return internal::WL_SetSides(
        hc_, one_sided ? Sides::kFrontSide : Sides::kBothSides);
  }

  /// @brief Legacy method to determine if the lidar is firing from one side.
  ///
  /// @param[out] one_sided Set to true unless shots are being fired from
  /// both sides of the device.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  WL_DEPRECATED("Use GetSides() instead.")
  virtual Status GetOneSided(bool* one_sided) {
    Sides sides;
    auto status = internal::WL_GetSides(hc_, &sides);
    if (status == Status::kOk) {
      *one_sided = sides != Sides::kBothSides;
      return Status::kOk;
    }
    return Status::kErrorInternal;
  }

  /// @brief Sets which side to fire the laser out of.
  ///
  /// By default, the lidar will fire shots from the front and back of the
  /// device, effectively doubling the update rate and horizontal resolution. In
  /// one-sided mode, the lidar will only fire shots from the front or back
  /// of the device.
  ///
  /// @param[in] sides A waymo::Sides enum specifying which side to fire from.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetSides(Sides sides) {
    return internal::WL_SetSides(hc_, sides);
  }

  /// @brief Gets which side of the lidar the laser is firing out of.
  ///
  /// @param[out] sides A waymo::Sides enum specifying which side of the device
  /// is firing from.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetSides(Sides* sides) {
    return internal::WL_GetSides(hc_, sides);
  }

  /// @brief Computes normals from point cloud data.
  ///
  /// The lidar can estimate surface normals by post-processing the return data.
  /// Enabling this feature makes additional data (normal vector, planarity,
  /// reflectivity, and the cosine of the angle of incidence) available with
  /// each Return.
  ///
  /// @param[in] enabled Set to true to enable normal computation. Defaults to
  /// `false`.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  WL_DEPRECATED("Use SetComputeNormals(Sides) instead.")
  virtual Status SetComputeNormals(bool enabled) {
    return internal::WL_SetComputeNormals(
        hc_, enabled ? Sides::kBothSides : Sides::kNone);
  }

  /// @brief Gets the normal computation setting.
  ///
  /// @param[out] enabled Set to true if normal computation is enabled.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  WL_DEPRECATED("Use GetComputeNormals(Sides *) instead.")
  virtual Status GetComputeNormals(bool* enabled) {
    Sides sides;
    auto status = internal::WL_GetComputeNormals(hc_, &sides);
    if (status == Status::kOk) {
      *enabled = sides != Sides::kNone;
      return Status::kOk;
    }
    return Status::kErrorInternal;
  }

  /// @brief Computes normals from point cloud data.
  ///
  /// The lidar can estimate surface normals by post-processing the return data.
  /// Enabling this feature makes additional data (normal vector, planarity,
  /// reflectivity, and the cosine of the angle of incidence) available with
  /// each Return.
  ///
  /// @param[in] sides A waymo::Sides enum specifying which side to enable
  /// normal computation.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetComputeNormals(Sides sides) {
    return internal::WL_SetComputeNormals(hc_, sides);
  }

  /// @brief Gets the normal computation setting.
  ///
  /// @param[out] sides A waymo::Sides enum specifying which sides have normal
  /// computation enabled.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetComputeNormals(Sides* sides) {
    return internal::WL_GetComputeNormals(hc_, sides);
  }

  /// @brief Sets the vertical shot table.
  ///
  /// Set the vertical angles where the device will fire shots.  The pitch_table
  /// is specified as a vector of angles in degrees.  The angles must be in the
  /// range of [20.75, -78.0] in descending order.  The difference
  /// between adjacent shots must be at least 0.75 degrees.
  ///
  ///     ______
  ///     |base|
  ///     +____+
  ///     |    | ---- 0 degrees
  ///     |    |
  ///     |____|
  ///       |    \ -45
  ///       |       degrees
  ///       -90 degrees
  ///
  /// @param[in] pitches A vector of angles in degrees.
  /// @param[in] interlaced If true, interleave shots between adjacent pitch
  /// table entries.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetPitchTable(std::vector<double> pitches, bool interlaced) {
    return internal::WL_SetPitchTable(hc_, pitches.data(), pitches.size(),
                                      interlaced);
  }

  /// @brief Gets the vertical shot table.
  ///
  /// Get the vertical angles where the device will fire shots.  The pitch_table
  /// is specified as a vector of angles in degrees.
  ///
  /// @param[out] pitches A pointer to a vector of angles in degrees.
  /// @param[out] interlaced A boolean indicating if interlaced mode is enabled.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetPitchTable(std::vector<double>* pitches, bool* interlaced) {
    int num_pitches;
    double* pitch_arr;
    Status status;
    if (pitches == nullptr || interlaced == nullptr) {
      return Status::kErrorInvalidArgument;
    }
    if ((status = internal::WL_GetPitchTable(hc_, &pitch_arr, &num_pitches,
                                             interlaced)) == Status::kOk) {
      *pitches = std::vector<double>(pitch_arr, pitch_arr + num_pitches);
      free(pitch_arr);
    }
    return status;
  }

  /// @brief Gets the limits for pitch tables on this device.
  ///
  /// Retrieves the minimum and maximum pitch angle supported by this device as
  /// well as the minimum spacing between adjacent shots.
  ///
  /// @param[out] limits A pointer to a waymo::PitchTableLimits structure.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetPitchTableLimits(PitchTableLimits* limits) {
    return internal::WL_GetPitchTableLimits(hc_, limits);
  }

  /// @brief Sets the maximum number of scans returned from GetLidarData.
  ///
  /// The number of scans returned from a call to GetLidarData. When set to
  /// zero, GetLidarData returns the scans that comprise an entire "spin"
  /// (360 degrees). You might choose to set `num_scans` to a smaller number if
  /// you want to interleave point cloud processing with the data collection.
  ///
  /// @param[in] num_scans The maximum number of scans returned from a single
  /// call of GetLidarData. Defaults to `0`.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetScansPerShard(int num_scans) {
    return internal::WL_SetScansPerShard(hc_, num_scans);
  }

  /// @brief Gets the number of scans per shard.
  ///
  /// @param[out] num_scans The maximum number of scans that will be returned by
  /// GetLidarData.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetScansPerShard(int* num_scans) {
    return internal::WL_GetScansPerShard(hc_, num_scans);
  }

  /// @brief Sets whether or not to include noise returns in scan data.
  ///
  /// Noise returns are returns that the driver considers to be artifacts
  /// rather than real returns.
  ///
  /// @param[in] enabled If true (the default), don't include
  /// noise returns in the point cloud returned by GetLidarData.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetDropNoiseReturns(bool enabled) {
    return internal::WL_SetDropNoiseReturns(hc_, enabled);
  }

  /// @brief Gets the drop noise returns setting.
  ///
  /// @param[out] enabled Set to true if noise returns are dropped
  /// from the point cloud.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetDropNoiseReturns(bool* enabled) {
    return internal::WL_GetDropNoiseReturns(hc_, enabled);
  }

  struct SystemError {
    // A stable identifier for this type of error.
    std::string code;

    // A human-readable description of the error.
    std::string description;

    // The time that this error occurred in seconds since the Unix Epoch.
    double timestamp;

    // The severity level of this error.
    Severity severity;

    SystemError() = default;
    SystemError(std::string c, std::string d, double t, Severity s)
        : code(c), description(d), timestamp(t), severity(s) {}
  };

  /// @brief Get system errors
  ///
  /// Returns a vector of errors that have accumulated since the last call
  /// to GetSystemErrors.  An empty vector is returned if no new errors
  /// have occurred.
  ///
  /// @param[out] system_errors A pointer to a vector of waymo::SystemError
  /// structures.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetSystemErrors(std::vector<SystemError>* system_errors) {
    int num_errors;
    WL_SystemError* wl_system_errors;
    Status status;

    if ((status = internal::WL_GetSystemErrors(hc_, &wl_system_errors,
                                               &num_errors)) == Status::kOk) {
      for (int i = 0; i < num_errors; ++i) {
        system_errors->emplace_back(
            wl_system_errors[i].code, wl_system_errors[i].description,
            wl_system_errors[i].timestamp, wl_system_errors[i].severity);
        free(static_cast<void*>(wl_system_errors[i].code));
        free(static_cast<void*>(wl_system_errors[i].description));
      }
      free(wl_system_errors);
    }
    return status;
  }

  struct SystemInfo {
    std::string ip_address;
    std::string mac_address;
    std::string model_number;
    std::string serial_number;
    std::string software_build_version;
    std::string software_build_date;
  };
  /// @brief Get system information
  ///
  /// @param[out] info A pointer to a waymo::SystemInfo structure.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetSystemInfo(SystemInfo* info) {
    waymo::WL_SystemInfo system_info;
    Status status = WL_GetSystemInfo(hc_, &system_info);
    if (status == Status::kOk) {
      info->ip_address = system_info.ip_address;
      free(system_info.ip_address);
      info->mac_address = system_info.mac_address;
      free(system_info.mac_address);
      info->model_number = system_info.model_number;
      free(system_info.model_number);
      info->serial_number = system_info.serial_number;
      free(system_info.serial_number);
      info->software_build_version = system_info.software_build_version;
      free(system_info.software_build_version);
      info->software_build_date = system_info.software_build_date;
      free(system_info.software_build_date);
    }
    return status;
  }

  /// @brief Get system status
  ///
  /// @param[out] system_status A pointer to a waymo::SystemStatus enum.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetSystemStatus(SystemStatus* system_status) {
    return internal::WL_GetSystemStatus(hc_, system_status);
  }

  /// @brief Gets the calibration parameters.
  ///
  /// @param[in] persist If true, read the calibration parameters from
  /// persistent storage, otherwise, return the calibration parameters currently
  /// in use.
  /// @param[out] cal A pointer to the Honeycomb calibration structure.
  ///
  /// @return Status::kOk if the query was successful, an error otherwise.
  virtual Status GetCalibrationParameters(bool persist, Calibration* cal) {
    return internal::experimental::WL_GetCalibrationParameters(hc_, persist,
                                                               cal);
  }

  /// @brief Sets the calibration parameters.
  ///
  /// @param[in] persist If true, write the calibration parameters to persistent
  /// storage.
  /// @param[in] cal A pointer to the Honeycomb calibration structure.
  ///
  /// @return Status::kOk if the setting was successful, an error otherwise.
  virtual Status SetCalibrationParameters(bool persist, Calibration* cal) {
    return internal::experimental::WL_SetCalibrationParameters(hc_, persist,
                                                               cal);
  }

 protected:
  internal::WL_LidarHandle* hc_ = nullptr;
};

}  // namespace waymo

#endif  // HONEYCOMB_H_
