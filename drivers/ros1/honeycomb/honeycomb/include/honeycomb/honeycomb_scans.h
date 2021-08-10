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
#ifndef HONEYCOMB_SCANS_H_
#define HONEYCOMB_SCANS_H_

#include <functional>
#include <memory>

#include "honeycomb/honeycomb_scan_data.h"

/// The Honeycomb::GetLidarData method returns the point cloud data in a Scans
/// structure. Scans consists of multiple Scan structures.  A Scan represents a
/// vertical sweep of laser shots. Each Shot represents a single
/// firing of the laser.  A single Shot may contain multiple Returns.
///
/// By default, Honeycomb::GetLidarData returns Scans that encompass a full 360
/// degree point cloud, but it is possible to get data at a higher frequency
/// that consist of partial spins, also known as "shards", by calling
/// Honeycomb::SetScansPerShard with a non-zero value.
///
/// An example of how to programmatically access the Scans data is as follows:
///
///     for (const auto& scan : scans->GetScans()) {
///       for (const auto& shot : scan) {
///         for (const auto& r : shot) {
///           const auto& coords = r.GetCoordinates();
///           std::cout << coords.x << "," << coords.y << "," << coords.z
///                     << "," << r.Intensity() << std::endl;
///         }
///       }
///     }
///
/// Alternatively, if you are not interested in the scan boundaries, you can
/// simplify the loop as follows:
///
///     for (const auto &shot : scans->GetShots()) {
///       for (const auto &r : shot) {
///         const auto& coords = r.GetCoordinates();
///         std::cout << coords.x << "," << coords.y << "," << coords.z << ","
///                   << r.Intensity() << std::endl;
///         }
///       }
///     }
namespace waymo {

/// @brief The Return class represents one of possibly multiple returns from a
/// single Shot.
class Return {
 public:
  explicit Return(const internal::LaserShotData& s, int i)
      : shot_(&s), idx_(i) {}

  /// @brief Returns the intensity of the return.
  ///
  /// A measure of intensity partly compensated for non-linearities in the
  /// receiver and distance effects. This intensity may allow a coarse
  /// comparison between objects in the scene.
  ///
  /// A value of 1.0 corresponds to the intensity of a return from a white
  /// Lambertian surface.
  double Intensity() const { return GetReturn().Intensity(); }

  /// @brief Returns the raw intensity of the return.
  ///
  /// A measure of intensity directly related to the amount of light received
  /// in a light pulse. This intensity may be the best one to use when
  /// attempting to find edges in the intensity data.
  ///
  /// This value is likely to vary significantly from device to device and
  /// between different generations of the device.
  double RawIntensity() const { return GetRawReturn().RawIntensity(); }

  /// @brief Returns the distance from the lidar to the point in meters.
  double RangeInMeters() const { return GetReturn().RangeInMeters(); }

  /// @brief Returns the width of the return pulse in meters.
  ///
  /// A coarse estimate of how long the pulse received by the lidar is. Can be
  /// used to flag returns that are due to multiple or diffuse objects, or that
  /// are at high incidence angles.
  double PulseWidthInMeters() const {
    return GetRawReturn().PulseWidthInMeters();
  }

  /// @brief Returns the elongation of the pulse beyond it's nominal width
  /// in meters.
  ///
  /// Lidar elongation refers to the elongation of the pulse beyond its nominal
  /// width. Returns with long pulse elongation, for example, indicate that the
  /// laser reflection is potentially smeared or refracted, such that the return
  /// pulse is elongated in time.  Most returns have an elongation of zero.
  double ElongationInMeters() const { return GetReturn().ElongationInMeters(); }

  /// @brief Returns XYZ coordinates of the point in the lidar's frame.
  Vec3d GetCoordinates() const { return GetReturn().GetCoordinates(); }

  /// @brief Returns the normal vector to the surface.
  ///
  /// Normals are only computed if SetComputeNormals has been enabled, and they
  /// are only computed for points shot from the front-side.  If normals
  /// cannot be computed, the values are set to all zeroes.
  Vec3d Normal() const { return GetNormal().Normal(); }

  /// @brief Returns the planarity.
  ///
  /// Planarity for each return of this shot based on the local neighborhood
  /// of returns around this shot, which is roughly equivalent of 1.0 -
  /// cos(local curvature). This is not measured but estimated
  /// using post-processing of the local normals. Values range from 0.0
  /// (non-planar) to 1.0 (perfectly planar).  If the normal is zero-length, the
  /// planarity will not be valid.
  double Planarity() const { return GetPlanarity().GetPlanarity(); }

  /// @brief Returns the cosine of the angle of incidence.
  ///
  /// A valid angle of incidence should sit within [0, 90 degrees), so its
  /// cosine value should sit within (0, 1]. Any invalid angle of incidence will
  /// have value zero. If the normal is zero-length, the cosine will not be
  /// valid.
  double CosAngleIncidence() const {
    return GetCosAngleIncidence().GetCosAngleIncidence();
  }

  /// @brief Returns the reflectivity of the surface.
  ///
  /// Reflectivity is the calibrated intensity corrected for angle of
  /// incidence using surface normals and assuming a Lambertian surface.
  ///
  /// Note: if the cosine of angle of incidence is invalid, we set the value
  /// to be kIndeterminateReflectivity.
  double Reflectivity() const {
    return CosAngleIncidence() == 0 ? kIndeterminateReflectivity
                                    : Intensity() / CosAngleIncidence();
  }

  /// @brief Returns whether or not the return has been classified as mixed.
  ///
  /// Mixed returns are returns that are longer than anticipated. This can occur
  /// when returns from several similar ranges get mixed together or by returns
  /// from a target that is seen at a high angle of incidence.
  bool IsMixed() const { return GetReturn().IsMixed(); }

  /// @brief Returns whether or not the return has been classified as noise.
  bool IsNoise() const { return GetReturn().IsNoise(); }

  /// @brief Returns the return index number.
  ///
  /// A single Shot is capable of generating multiple returns.  The return index
  /// enumerates multiple returns from the same shot. Returns are indexed
  /// starting at zero.
  int Index() const { return idx_; }

 private:
  static constexpr double kIndeterminateReflectivity = 1e5;
  const internal::LaserShotData* shot_;
  int idx_;

  const internal::CalibratedReturnData& GetReturn() const {
    return shot_->GetReturn(idx_);
  }
  const internal::RawReturnData& GetRawReturn() const {
    return shot_->GetRawReturn(GetReturn().GetRawIndex());
  }
  const internal::ShotNormal& GetNormal() const {
    return shot_->GetNormal(idx_);
  }
  const internal::Planarity& GetPlanarity() const {
    return shot_->GetPlanarity(idx_);
  }
  const internal::CosAngleIncidence& GetCosAngleIncidence() const {
    return shot_->GetCosAngleIncidence(idx_);
  }
};

/// @cond
template <typename T>
class IteratorPair {
 public:
  IteratorPair(T&& b, T&& e) : b_{std::forward<T>(b)}, e_{std::forward<T>(e)} {}
  T begin() const { return b_; }
  T end() const { return e_; }

 private:
  T b_, e_;
};

class ScansShotIterator;
class ScansScanIterator;
/// @endcond

class ShotReturnIterator;
typedef enum class ReturnMode {
  kAllReturns,
  kFirstReturn,
  kLastReturn,
  kStrongestReturn,
} ReturnMode;

/// @brief The Shot class contains the data from a single pulse of the laser.
class Shot {
 public:
  explicit Shot(const internal::LaserShotData& shot,
                const internal::AngleInfoData& angle)
      : shot_(&shot), angle_(&angle) {}

  /// @brief Returns the number of Return structures for this Shot.
  int NumReturns() const { return shot_->NumCalibratedReturns(); }

  /// @brief Returns the yaw and pitch of the Shot in radians.
  ///
  /// The yaw value is in the range of [-pi, pi].  The pitch value is in the
  /// range [pi/6, -pi/2].
  const LaserSteeringAnglesRadians GetBeamAnglesInRadians() const {
    return angle_->GetBeamAnglesInRadians();
  }

  /// @brief Returns the yaw and pitch of the Shot in degrees.
  ///
  /// The yaw value is in the range of [-180, 180].  The pitch value is in the
  /// range [30, -90].
  const LaserSteeringAnglesDegrees GetBeamAnglesInDegrees() const {
    return angle_->GetBeamAnglesInDegrees();
  }

  /// @brief Returns which side of the device the shot was taken from.
  ///
  /// A value of 0 is returned for shots taken from the front side of the device
  /// and a value of 1 is returned for shots from the back side.
  int BeamSide() const { return angle_->BeamSide(); }

  /// @brief Returns max possible value for pitch.
  double PitchSpanRadians() const { return angle_->PitchSpanRadians(); }

  /// @brief Gets the specified Return from the Shot.
  ///
  /// @param[in] i The index of the requested Return.
  const Return GetReturn(int i) const { return Return(*shot_, i); }

  /// @brief Returns an iterator to the first Return of the Shot.
  ShotReturnIterator begin() const;

  /// @brief Returns an iterator beyond the last Return of the Shot.
  ShotReturnIterator end() const;

  /// @brief Returns an iterator to the first Return of the Shot, based on mode.
  ShotReturnIterator BeginReturns(ReturnMode mode) const;

  /// @brief Returns an iterator beyond the last Return of the Shot, based on
  /// mode.
  ShotReturnIterator EndReturns(ReturnMode mode) const;

  /// @brief Returns an iterator pair for range-based for-loops of Returns in
  /// Shot.
  ///
  /// Example usage:
  ///   for (const auto& r : shot.GetReturns(ALL_RETURNS)) {
  ///     // process return
  ///   }
  IteratorPair<ShotReturnIterator> GetReturns(ReturnMode mode) const;

 private:
  const internal::LaserShotData* shot_;
  const internal::AngleInfoData* angle_;
};

class ScanShotIterator;

/// @brief The Scan class contains a set of Shot structures from a single
/// vertical sweep of the laser.
class Scan {
 public:
  explicit Scan(const internal::ScanData& scan) : scan_(&scan) {}

  /// @brief Returns the number of shots in this Scan.
  int NumShots() const { return scan_->NumShots(); }

  /// @brief Gets the specified Shot from the Scan.
  ///
  /// @param[in] i The index of the requested Shot.
  const Shot GetShot(int i) const {
    return Shot(scan_->GetShot(i), scan_->GetAngle(i));
  }

  /// @brief Returns true if an error was detected while capturing the scan.
  const double IsError() const { return scan_->IsError(); }

  /// @brief Returns the time at which the first shot in the scan was taken, in
  /// seconds since the Unix epoch.
  const double Timestamp() const { return scan_->Timestamp(); }

  /// @brief Returns an iterator to the first Shot of the Scan.
  ScanShotIterator begin() const;

  /// @brief Returns an iterator beyond the last Shot of the Scan.
  ScanShotIterator end() const;

 private:
  const internal::ScanData* scan_;
};

/// @brief The Scans class contains a "shard" of Scan structures returned from
/// GetLidarData.
class Scans {
 public:
  explicit Scans(const internal::ScansData& scans) : scans_(scans) {}

  /// @brief Returns the number of Scan structures in this shard.
  int NumScans() const { return scans_.NumScans(); }

  /// @brief Returns the maximum number of Shot structures in a single Scan.
  int MaxShotsPerScan() const { return internal::ScanData::kMaxShotsPerScan; }

  /// @brief Returns the maximum number of Return structures in a single Shot.
  int MaxReturnsPerShot() const {
    return internal::LaserShotData::kMaxNumReturns;
  }

  /// @brief Gets the specified Scan from the shard.
  const Scan GetScan(int i) const { return Scan(scans_.GetScan(i)); }

  /// @brief Returns an iterator to the first Shot in Scans.
  ScansShotIterator BeginShots() const;

  /// @brief Returns an iterator beyond the last Shot in Scans.
  ScansShotIterator EndShots() const;

  /// @brief Returns an iterator pair for range-based for-loops of Shots in
  /// Scans.
  ///
  /// Example usage:
  ///   for (const auto& shot : scans.GetShots()) {
  ///     // process shot
  ///   }
  IteratorPair<ScansShotIterator> GetShots() const;

  /// @brief Returns an iterator to the first Scan in Scans.
  ScansScanIterator BeginScans() const;

  /// @brief Returns an iterator beyond the last Scan in Scans.
  ScansScanIterator EndScans() const;

  /// @brief Returns an iterator pair for range-based for-loops of Scan
  /// structures in Scans.
  ///
  /// Example usage:
  ///   for (const auto& scan : scans.GetScans()) {
  ///     // process scan
  ///   }
  IteratorPair<ScansScanIterator> GetScans() const;

  /// @brief Returns the precision with which the Pitch is measured, in  Radians
  static constexpr double PitchPrecisionRadians() {
    return internal::AngleInfoData::PitchPrecisionRadians();
  }

 private:
  const internal::ScansData& scans_;
};

using ScansPtr = std::unique_ptr<Scans, std::function<void(Scans*)>>;

/// @cond
class ScansShotIterator
    : public std::iterator<std::input_iterator_tag, const Shot> {
 public:
  explicit ScansShotIterator(const Scans& s, bool begin = true)
      : scans_(s),
        scan_index_(begin ? 0 : s.NumScans()),
        shot_index_(0),
        scan_(s.GetScan(scan_index_)) {}

  ScansShotIterator& operator++();

  bool operator==(const ScansShotIterator& rhs) const {
    return (scan_index_ == rhs.scan_index_) &&
           (shot_index_ == rhs.shot_index_) && (&scans_ == &rhs.scans_);
  }

  bool operator!=(const ScansShotIterator& rhs) const {
    return !(*this == rhs);
  }

  const Shot operator*() const { return scan_.GetShot(shot_index_); }

 private:
  const Scans& scans_;
  int scan_index_;
  int shot_index_;
  Scan scan_;
};
/// @endcond

/// @cond
class ScansScanIterator
    : public std::iterator<std::input_iterator_tag, const Scan> {
 public:
  explicit ScansScanIterator(const Scans& s, bool begin = true)
      : scans_(s), scan_index_(begin ? 0 : s.NumScans()) {}

  ScansScanIterator& operator++() {
    scan_index_++;
    return *this;
  }

  bool operator==(const ScansScanIterator& rhs) const {
    return (scan_index_ == rhs.scan_index_) && (&scans_ == &rhs.scans_);
  }

  bool operator!=(const ScansScanIterator& rhs) const {
    return !(*this == rhs);
  }

  const Scan operator*() const { return scans_.GetScan(scan_index_); }

 private:
  const Scans& scans_;
  int scan_index_;
};
/// @endcond

/// @cond
class ScanShotIterator
    : public std::iterator<std::input_iterator_tag, const Shot> {
 public:
  explicit ScanShotIterator(const Scan& s, bool begin = true)
      : scan_(s), shot_index_(begin ? 0 : s.NumShots()) {}

  ScanShotIterator& operator++() {
    shot_index_++;
    return *this;
  }

  bool operator==(const ScanShotIterator& rhs) const {
    return (shot_index_ == rhs.shot_index_) && (&scan_ == &rhs.scan_);
  }

  bool operator!=(const ScanShotIterator& rhs) const { return !(*this == rhs); }

  const Shot operator*() const { return scan_.GetShot(shot_index_); }

 private:
  const Scan& scan_;
  int shot_index_;
};
/// @endcond

/// @cond
class ShotReturnIterator
    : public std::iterator<std::input_iterator_tag, const Return> {
 public:
  // Creates an iterator over returns in a shot.
  // Args:
  //  s - the shot to iterate over.
  //  begin - a boolean indicating if the iterator should point to the
  //          first return, or beyond the last return.
  //  mode - the return mode, one of [all returns, first return, last return,
  //         or strongest return].
  explicit ShotReturnIterator(const Shot& s, bool begin = true,
                              ReturnMode mode = ReturnMode::kAllReturns)
      : shot_(s) {
    if (s.NumReturns() == 0) {
      return_index_ = 0;
    } else {
      switch (mode) {
        case ReturnMode::kAllReturns:
          return_index_ = begin ? 0 : s.NumReturns();
          break;
        case ReturnMode::kFirstReturn:
          return_index_ = begin ? 0 : 1;
          break;
        case ReturnMode::kLastReturn:
          return_index_ = begin ? s.NumReturns() - 1 : s.NumReturns();
          break;
        case ReturnMode::kStrongestReturn:
          int strongest = 0;
          double strongest_intensity = -1;
          for (int i = 0; i < s.NumReturns(); ++i) {
            double this_intensity = s.GetReturn(i).Intensity();
            if (this_intensity > strongest_intensity) {
              strongest = i;
              strongest_intensity = this_intensity;
            }
          }
          return_index_ = begin ? strongest : strongest + 1;
          break;
      }
    }
  }

  ShotReturnIterator& operator++() {
    return_index_++;
    return *this;
  }

  bool operator==(const ShotReturnIterator& rhs) const {
    return (&shot_ == &rhs.shot_) && (return_index_ == rhs.return_index_);
  }

  bool operator!=(const ShotReturnIterator& rhs) const {
    return !(*this == rhs);
  }

  const Return operator*() const { return shot_.GetReturn(return_index_); }

 private:
  const Shot& shot_;
  int return_index_;
};
/// @endcond

inline ScansShotIterator Scans::BeginShots() const {
  return ScansShotIterator(*this, true);
}
inline ScansShotIterator Scans::EndShots() const {
  return ScansShotIterator(*this, false);
}
inline IteratorPair<ScansShotIterator> Scans::GetShots() const {
  return {BeginShots(), EndShots()};
}
inline ScansShotIterator& ScansShotIterator::operator++() {
  shot_index_++;
  if (shot_index_ == scan_.NumShots()) {
    // Increment to the new scan and reset shot.
    scan_index_++;
    shot_index_ = 0;
    scan_ = scans_.GetScan(scan_index_);
  }
  return *this;
}

inline ScansScanIterator Scans::BeginScans() const {
  return ScansScanIterator(*this, true);
}
inline ScansScanIterator Scans::EndScans() const {
  return ScansScanIterator(*this, false);
}
inline IteratorPair<ScansScanIterator> Scans::GetScans() const {
  return {BeginScans(), EndScans()};
}

inline ScanShotIterator Scan::begin() const {
  return ScanShotIterator(*this, true);
}
inline ScanShotIterator Scan::end() const {
  return ScanShotIterator(*this, false);
}

inline ShotReturnIterator Shot::begin() const {
  return ShotReturnIterator(*this, true);
}
inline ShotReturnIterator Shot::end() const {
  return ShotReturnIterator(*this, false);
}

inline ShotReturnIterator Shot::BeginReturns(ReturnMode mode) const {
  return ShotReturnIterator(*this, true, mode);
}
inline ShotReturnIterator Shot::EndReturns(ReturnMode mode) const {
  return ShotReturnIterator(*this, false, mode);
}
inline IteratorPair<ShotReturnIterator> Shot::GetReturns(
    ReturnMode mode) const {
  return {BeginReturns(mode), EndReturns(mode)};
}
}  // namespace waymo

#endif  // HONEYCOMB_SCANS_H_
