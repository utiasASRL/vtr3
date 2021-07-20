/*
 * Unpublished Work Copyright 2020 Waymo LLC.  All rights reserved.
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
#include "hc/scan_image_producer.h"

#include <algorithm>

namespace honeycomb {
namespace {
static constexpr double kMaxRange = 50.0;
static constexpr double kMaxIntensity = 12.0;
static constexpr double kMaxPulseWidth = 5.0;  // 5m (31 * 16cm) max possible
}  // namespace

void ScanImageProducer::SetConfig(const hc::HcNodeConfig &config,
                                  const std::vector<double> &pitches) {
  pitches_ = pitches;
  frame_id_ = config.frame_id;
  yaw_between_scans_ =
      360.0 / (config.vertical_scan_frequency / config.spin_frequency);
  width_ = floor(config.fov / yaw_between_scans_) + 1;
  height_ = pitches_.size();
  byte_depth_ = config.si_mono_bits == 8 ? 1 : 2;
  max_val_ = config.si_mono_bits == 8 ? std::numeric_limits<uint8_t>::max()
                                      : std::numeric_limits<uint16_t>::max();
  // 2 values reserved, 0 for dropped, 1 for no return
  max_val_range_ = max_val_ - 2;
  return_mode_ = static_cast<waymo::ReturnMode>(config.si_return);
  sides_ = static_cast<waymo::Sides>(config.sides);
  direction_ = config.direction;
  fov_ = config.fov;
}

sensor_msgs::ImagePtr ScanImageProducer::ScansToImage(
    const waymo::Scans *scans, const hc::HcNodeConfig &config,
    const std::vector<double> &pitches, ImageType type) {
  SetConfig(config, pitches);
  if (width_ == 0 && height_ == 0 && scans != nullptr) {
    // May not be initialized, nothing to do
    return nullptr;
  }
  sensor_msgs::ImagePtr out_img(new sensor_msgs::Image());
  out_img->header.frame_id = frame_id_;
  out_img->width = width_;
  out_img->height = height_;
  out_img->step = width_ * byte_depth_;
  out_img->encoding = byte_depth_ == 1 ? "mono8" : "mono16";
  out_img->data.resize(out_img->step * height_);
  out_img->is_bigendian = waymo::kIsBigEndianHost;
  uint16_t *data16 = reinterpret_cast<uint16_t *>(&out_img->data[0]);

  if (scans->NumScans() == 0) {
    return nullptr;
  }

  int h_index = 0;
  int w_index = -1;
  pitch_precision_degrees_ = waymo::Rad2Deg(scans->PitchPrecisionRadians());

  int first_usable_scan = -1;
  for (int i = 0; first_usable_scan < 0 && i < scans->NumScans(); i++) {
    if (scans->GetScan(i).NumShots() > 0) first_usable_scan = i;
  }
  if (first_usable_scan < 0) {
    return nullptr;
  }

  yaw_per_pitch_ =
      yaw_between_scans_ /
      waymo::Rad2Deg(
          scans->GetScan(first_usable_scan).GetShot(0).PitchSpanRadians());

  const waymo::LaserSteeringAnglesDegrees &spin_first_angles =
      scans->GetScan(first_usable_scan).GetShot(0).GetBeamAnglesInDegrees();
  const double spin_phase = fmod(
      spin_first_angles.yaw_deg - spin_first_angles.pitch_deg * yaw_per_pitch_,
      yaw_between_scans_);

  for (const waymo::Scan &scan : scans->GetScans()) {
    // Using only one beam side, skip back side when both is enabled
    if (sides_ == waymo::Sides::kBothSides && scan.GetShot(0).BeamSide() == 1) {
      continue;
    }

    // Resetting w_index from ever scan,
    // to be calculated once for first shot with valid pitch
    w_index = -1;

    for (const waymo::Shot &shot : scan) {
      const waymo::LaserSteeringAnglesDegrees &angle =
          shot.GetBeamAnglesInDegrees();

      // Get image intensity value from shot return
      unsigned int val;
      if (shot.NumReturns() > 0) {
        // Select return to use based on strategy chosen
        const waymo::Return &r = *shot.GetReturns(return_mode_).begin();

        // Get value and scale
        double tmp_val = GetValue(r, type);
        if (tmp_val == -1) {
          return nullptr;  // type is wrong
        }
        val = static_cast<unsigned int>(tmp_val * max_val_range_);
        val = std::max(2u, val + 2u);  // 0 : dropped frame 1: no return
        val = std::min(val, max_val_);
      } else {
        val = 1u;  // no return
      }

      // Calculating h_index from pitch
      h_index = GetHeightIndex(angle.pitch_deg, scan.GetShot(0).BeamSide());

      if (h_index < 0) {
        continue;  // no valid pitch index, dropping shot
      }

      // Calculating w_index from yaw and pitch, only if pitch is valid
      if (w_index == -1) {
        w_index = GetWidthIndex(angle.yaw_deg, angle.pitch_deg, spin_phase);
        if (w_index == -1) {
          break;  // drop scan with w_index out of range/FoV.
        }
      }

      // Fill pixel based on byte_depth
      if (byte_depth_ == 1) {
        out_img->data[h_index * width_ + w_index] = static_cast<uint8_t>(val);
      } else if (byte_depth_ == 2) {
        data16[h_index * width_ + w_index] = static_cast<uint16_t>(val);
      }
    }
  }


  // we use the mean time of start and end of scans as point cloud timestamp
  // as long as there is no motion compensation for translation (de-skewing)
  const double start = scans->GetScan(0).Timestamp();
  const double end = scans->GetScan(scans->NumScans() - 1).Timestamp();
  out_img->header.stamp.fromSec(start + (end - start) / 2);
  return out_img;
}

double ScanImageProducer::GetValue(const waymo::Return &r,
                                   ImageType type) const {
  double val;

  switch (type) {
    case ImageType::kPulseWidth:
      val = std::min(kMaxPulseWidth, r.PulseWidthInMeters()) / kMaxPulseWidth;
      break;
    case ImageType::kIntensity:
      val = std::min(kMaxIntensity, r.Intensity()) / kMaxIntensity;
      break;
    case ImageType::kRange:
      val = std::min(kMaxRange, r.RangeInMeters()) / kMaxRange;
      break;
    default:
      return -1;
  }
  return val;
}

int ScanImageProducer::GetHeightIndex(double pitch_deg, int beam_side) {
  int h_index_next = beam_side == 1 ? h_index_prev_ - 1 : h_index_prev_ + 1;

  // Check if h_index_next is correct for pitch_deg
  if (h_index_next >= 0 && h_index_next < pitches_.size()) {
    const double pitch_angle_dist = fabs(pitches_[h_index_next] - pitch_deg);
    if (pitch_angle_dist <= 2.0 * pitch_precision_degrees_) {
      h_index_prev_ = h_index_next;
      return h_index_next;
    }
  }

  // Do full search and h_index_next does not match pitch_deg
  const auto lower_pitch = std::lower_bound(
      pitches_.begin(), pitches_.end(),
      pitch_deg + 2.0 * pitch_precision_degrees_, std::greater<double>());
  if (lower_pitch != pitches_.end()) {
    h_index_next = std::distance(pitches_.begin(), lower_pitch);
    const double pitch_angle_dist = fabs(pitches_[h_index_next] - pitch_deg);
    if (pitch_angle_dist <= 2.0 * pitch_precision_degrees_) {
      h_index_prev_ = h_index_next;
      return h_index_next;
    }
  }
  return -1;  // pitch_deg did not match any angle in pitches_
}

int ScanImageProducer::GetWidthIndex(double yaw_deg, double pitch_deg,
                                     double spin_phase) const {
  int w_index = -1;
  const double pitch_phase_compensated_yaw =
      (yaw_deg - pitch_deg * yaw_per_pitch_) - spin_phase;
  const double yaw_from_zero = fmod(
      (pitch_phase_compensated_yaw + fov_ / 2.0 - direction_) + 360.0, 360.0);
  w_index = std::round(yaw_from_zero / yaw_between_scans_);
  if (w_index < 0 || w_index >= width_) {
    return -1;
  }
  return w_index;
}

}  // namespace honeycomb
