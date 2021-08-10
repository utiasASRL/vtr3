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
#ifndef HONEYCOMB_SCAN_DATA_H_
#define HONEYCOMB_SCAN_DATA_H_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#if defined(__x86_64__)
#include <x86intrin.h>
#endif

namespace waymo {

struct Vec3d {
  double x;
  double y;
  double z;
};

struct Vec3i {
  int x;
  int y;
  int z;
};

struct LaserSteeringAnglesRadians {
  LaserSteeringAnglesRadians(double yaw_rad_in, double pitch_rad_in)
      : yaw_rad(yaw_rad_in), pitch_rad(pitch_rad_in) {}

  // Yaw steering angle, in radians.
  double yaw_rad;

  // Pitch steering angle, in radians.
  double pitch_rad;
};

struct LaserSteeringAnglesDegrees {
  LaserSteeringAnglesDegrees(double yaw_deg_in, double pitch_deg_in)
      : yaw_deg(yaw_deg_in), pitch_deg(pitch_deg_in) {}

  explicit LaserSteeringAnglesDegrees(const LaserSteeringAnglesRadians& a)
      : LaserSteeringAnglesDegrees(a.yaw_rad * 180 / M_PI,
                                   a.pitch_rad * 180 / M_PI) {}

  // Yaw steering angle, in degrees.
  double yaw_deg;

  // Pitch steering angle, in degrees.
  double pitch_deg;
};

namespace internal {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-private-field"
#endif

template <typename T>
class CompactUnitVector {
 public:
  double x() const { return to_double(x_raw_); }
  double y() const { return to_double(y_raw_); }
  double z() const { return to_double(z_raw_); }

  Vec3d ToVec3d() const { return Vec3d{x(), y(), z()}; }
  constexpr CompactUnitVector() : x_raw_(0), y_raw_(0), z_raw_(0) {}

 private:
  T x_raw_;
  T y_raw_;
  T z_raw_;

  static double to_double(T value) {
    return static_cast<double>(value) / std::numeric_limits<T>::max();
  }
} __attribute__((packed));

class RawReturnData {
 private:
  uint16_t range_;
  uint16_t pulse_width_ : 10;
  uint16_t peak_ : 10;
  uint8_t synthetic_peak_ : 1;
  uint8_t : 3;

 public:
  static constexpr int kMaxPeakValue = 1023;
  static constexpr double kPeakToFraction = 1.0 / kMaxPeakValue;
  static constexpr double kSpeedOfLightDividedByTwo = 149896229.;
  static constexpr int kRangePer5ns = 128;
  static constexpr int kRangePer10ns = kRangePer5ns * 2;
  static constexpr double kRangeToSeconds = 5e-9 / kRangePer5ns;
  static constexpr double kRangeToMeters =
      kRangeToSeconds * kSpeedOfLightDividedByTwo;
  double RawIntensity() const { return peak_ * kPeakToFraction; }
  double PulseWidthInMeters() const { return pulse_width_ * kRangeToMeters; }
} __attribute__((packed));

class CalibratedReturnData {
 private:
  uint16_t range_;
  uint16_t integer_intensity_;
  union {
    struct {
      int32_t x_raw_ : 24;
      int32_t y_raw_ : 24;
      int32_t z_raw_ : 24;
    } __attribute__((packed));
    uint8_t xyz_raw_bytes_[9];
  };

  uint8_t raw_index_ : 3;
  uint8_t mixed_ : 1;
  uint8_t weak_ : 1;
  uint8_t noise_ : 1;
  uint8_t self_return_ : 1;
  uint8_t range_alias_ : 1;
  uint8_t elongation_;
  uint8_t padding_;

  Vec3i GetRawCoordinates() const {
#if defined(__GNUC__) && defined(__SSSE3__) && !defined(ADDRESS_SANITIZER) && \
    !defined(THREAD_SANITIZER)
    const char xFF = '\xFF';
    __m128i vec =
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(xyz_raw_bytes_));
    const __m128i mask = _mm_set_epi8(xFF, xFF, xFF, xFF, 8, 7, 6, xFF, 5, 4, 3,
                                      xFF, 2, 1, 0, xFF);
    vec = _mm_shuffle_epi8(vec, mask);
    vec = _mm_srai_epi32(vec, 8);
    const int x = _mm_cvtsi128_si32(vec);
    const int y = _mm_cvtsi128_si32(_mm_shuffle_epi32(vec, 1));
    const int z = _mm_cvtsi128_si32(_mm_shuffle_epi32(vec, 2));
    return {x, y, z};
#else
    return {x_raw_, y_raw_, z_raw_};
#endif
  }

 public:
  Vec3d GetCoordinates() const {
    static constexpr double kCoordinateMetersPerTick = 1e-4;
    const Vec3i raw_coord = GetRawCoordinates();
    return {raw_coord.x * kCoordinateMetersPerTick,
            raw_coord.y * kCoordinateMetersPerTick,
            raw_coord.z * kCoordinateMetersPerTick};
  }

  double Intensity() const {
    uint32_t source = static_cast<uint32_t>(integer_intensity_) << 16;
    float dest;
    memcpy(static_cast<void*>(std::addressof(dest)),
           static_cast<const void*>(std::addressof(source)), sizeof(dest));
    return static_cast<double>(dest);
  }

  double RangeInMeters() const {
    return range_ * RawReturnData::kRangeToMeters;
  }

  double ElongationInMeters() const {
    return elongation_ * RawReturnData::kRangeToMeters;
  }

  bool IsMixed() const { return mixed_; }
  bool IsNoise() const { return noise_; }

  uint8_t GetRawIndex() const { return raw_index_; }
} __attribute__((packed));

class ShotNormal : CompactUnitVector<int16_t> {
 public:
  Vec3d Normal() const { return ToVec3d(); }
} __attribute__((packed));

class Planarity {
 private:
  uint8_t planarity_;

 public:
  double GetPlanarity() const {
    return static_cast<double>(planarity_) /
           std::numeric_limits<uint8_t>::max();
  }
  constexpr Planarity() : planarity_(0) {}
} __attribute__((packed));

class CosAngleIncidence {
 private:
  uint16_t value_;

 public:
  double GetCosAngleIncidence() const {
    return static_cast<double>(value_) / std::numeric_limits<uint16_t>::max();
  }
  constexpr CosAngleIncidence() : value_(0) {}
} __attribute__((packed));

static constexpr ShotNormal kZeroNormal;
static constexpr Planarity kZeroPlanarity;
static constexpr CosAngleIncidence kZeroCosAngleIncidence;

class LaserShotData {
 public:
  static constexpr int kMaxNumReturns = 3;

 private:
  static constexpr int kMaxNumRawReturns = 6;
  static constexpr int kMaxNumReturnNormals = 2;

  uint8_t num_raw_returns_;
  RawReturnData raw_returns_[kMaxNumRawReturns];

  CalibratedReturnData calibrated_returns_[kMaxNumReturns];

  uint32_t raw_timestamp_;

  uint16_t max_range_;

  CompactUnitVector<int16_t> direction_;
  ShotNormal normals_[kMaxNumReturnNormals];

  Planarity planarities_[kMaxNumReturnNormals];

  CosAngleIncidence cos_angle_incidence_values_[kMaxNumReturnNormals];

  uint8_t num_calibrated_returns_ : 3;
  uint8_t num_additional_projected_returns_ : 2;

 public:
  uint8_t NumCalibratedReturns() const { return num_calibrated_returns_; }
  const CalibratedReturnData& GetReturn(int i) const {
    return calibrated_returns_[i];
  }
  const RawReturnData& GetRawReturn(int i) const { return raw_returns_[i]; }
  const ShotNormal& GetNormal(int i) const {
    return i < kMaxNumReturnNormals ? normals_[i] : kZeroNormal;
  }
  const Planarity& GetPlanarity(int i) const {
    return i < kMaxNumReturnNormals ? planarities_[i] : kZeroPlanarity;
  }
  const CosAngleIncidence& GetCosAngleIncidence(int i) const {
    return i < kMaxNumReturnNormals ? cos_angle_incidence_values_[i]
                                    : kZeroCosAngleIncidence;
  }
} __attribute__((packed));

enum class HoneycombGeneration : uint8_t {
  kGen1 = 1,
  kGen2 = 2,
};

class AngleInfoData {
 private:
  uint16_t raw_yaw_;
  uint16_t raw_pitch_;
  HoneycombGeneration gen_ : 4;
  uint8_t padding_ : 4;

  static constexpr double kRawAngleUnitInRadians = (2.0 * M_PI) / 65536.;
  static constexpr double kPitchZeroOffset = (4.0 * M_PI) / 24;

  int PitchConversionModulus() const {
    return gen_ == HoneycombGeneration::kGen2 ? 4 : 3;
  }

  double BeamPitchInRadians() const {
    return fmod(raw_pitch_ * PitchPrecisionRadians(), PitchSpanRadians()) -
           kPitchZeroOffset;
  }

  double TurretYawInRadians() const {
    return raw_yaw_ * kRawAngleUnitInRadians;
  }

 public:
  static constexpr double PitchPrecisionRadians() {
    return 2.0 * kRawAngleUnitInRadians;
  }

  int BeamSide() const {
    return gen_ == HoneycombGeneration::kGen2 ? 0
           : BeamPitchInRadians() > M_PI / 2  ? 1
                                              : 0;
  }

  double PitchSpanRadians() const {
    return (4 * M_PI) / PitchConversionModulus();
  }

  const LaserSteeringAnglesRadians GetBeamAnglesInRadians() const {
    const double beam_pitch = BeamPitchInRadians();
    const double yaw = TurretYawInRadians();
    if (beam_pitch > M_PI / 2) {
      return LaserSteeringAnglesRadians(yaw - M_PI, beam_pitch - M_PI);
    } else {
      return LaserSteeringAnglesRadians(yaw > M_PI ? yaw - M_PI * 2 : yaw,
                                        -beam_pitch);
    }
  }
  const LaserSteeringAnglesDegrees GetBeamAnglesInDegrees() const {
    return LaserSteeringAnglesDegrees(GetBeamAnglesInRadians());
  }
} __attribute__((packed));

struct ChannelData {
 private:
  uint8_t padding[4];
} __attribute__((packed));

enum LaserScanDirection { SCAN_DIR_NEGATIVE = 0, SCAN_DIR_POSITIVE = 1 };

class ScanData {
 public:
  static constexpr int kMaxShotsPerScan = 192;

 private:
  uint16_t num_shots_;
  LaserShotData shots_[kMaxShotsPerScan];
  AngleInfoData angles_[kMaxShotsPerScan];
  ChannelData channels_[kMaxShotsPerScan];

  LaserScanDirection vertical_dir_;

  uint8_t padding_[96];
  double timestamp_;

  uint8_t padding2_[104];
  Vec3d sensor_location_;

  uint32_t sequence_num_;
  bool valid_feedback_;
  bool error_;
  bool dim_;
  int waveform_processor_version_;
  uint8_t boost_clocks_;
  uint8_t padding3_[180];

 public:
  bool IsError() const { return error_; }
  double Timestamp() const { return timestamp_; }
  uint16_t NumShots() const { return num_shots_; }
  const LaserShotData& GetShot(int i) const { return shots_[i]; }
  const AngleInfoData& GetAngle(int i) const { return angles_[i]; }
};

class ScansData {
 private:
  uint8_t padding0_[16];

  const ScanData* scans_;
  int num_scans_;
  bool spin_completed_;

  uint8_t padding1_[43];

 public:
  ScansData(const ScanData* scans, int num_scans, bool spin_completed)
      : scans_(scans), num_scans_(num_scans), spin_completed_(spin_completed) {}
  int NumScans() const { return num_scans_; }
  const ScanData& GetScan(int i) const { return scans_[i]; }
  bool IsSpinCompleted() const { return spin_completed_; }
};
#ifdef __clang__
#pragma clang diagnostic pop
#endif

}  // namespace internal
}  // namespace waymo

#endif  // HONEYCOMB_SCAN_DATA_H_
