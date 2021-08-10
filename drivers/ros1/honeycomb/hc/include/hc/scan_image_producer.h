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
#ifndef SCAN_IMAGE_PRODUCER_H
#define SCAN_IMAGE_PRODUCER_H

#include <sensor_msgs/Image.h>

#include "honeycomb/honeycomb.h"
#include "hc/HcNodeConfig.h"

namespace honeycomb {

// This class can convert waymo::Scans to Image and caches needed config.
// SetConfig() is used to set/update configeration which is cached.
// ScansToImage() can be used to get Image from Scan, like range image.
// Example:
//   std::unique_ptr<ScanImageProducer> si =
//   std::unique_ptr<ScanImageProducer>(new ScanImageProducer(config,
//   pitch_table)); sensor_msgs::ImagePtr range_image =
//     scan_image_producer_.ScansToImage(scans.get(),
//     ScanImageProducer::ImageType::kRange);
class ScanImageProducer {
 public:
  enum class ImageType { kRange = 0, kIntensity, kPulseWidth };

  sensor_msgs::ImagePtr ScansToImage(const waymo::Scans *scans,
                                     const hc::HcNodeConfig &config,
                                     const std::vector<double> &pitches,
                                     ImageType type = ImageType::kRange);

 private:
  void SetConfig(const hc::HcNodeConfig &config,
                 const std::vector<double> &pitches);
  double GetValue(const waymo::Return &r, ImageType type) const;
  int GetHeightIndex(double pitch_deg,
                     int beam_side);  // Shot beam_side, 0 for front, 1 for back
  int GetWidthIndex(double yaw_deg, double pitch_deg, double spin_phase) const;

  // Image output
  int width_{0};
  int height_{0};
  std::string frame_id_;
  int byte_depth_;
  waymo::Sides sides_;

  // Return selection
  waymo::ReturnMode return_mode_;

  // Return value type selection and scaling
  unsigned int max_val_;
  unsigned int max_val_range_;

  // Pitch to Image height index
  std::vector<double> pitches_;
  double pitch_precision_degrees_;
  int h_index_prev_;

  // Yaw to Image width index
  double direction_;
  double fov_;
  double yaw_between_scans_;
  double yaw_per_pitch_;
};

}  // namespace honeycomb

#endif  // SCAN_IMAGE_PRODUCER_H
