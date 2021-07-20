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
#include "hc/scans.h"

namespace honeycomb {

namespace {
int GetSizeForDataType(uint8_t datatype) {
  const std::map<int, int> kDataTypeSizeMap = {
      {sensor_msgs::PointField::INT8, 1},
      {sensor_msgs::PointField::UINT8, 1},
      {sensor_msgs::PointField::INT16, 2},
      {sensor_msgs::PointField::UINT16, 2},
      {sensor_msgs::PointField::INT32, 4},
      {sensor_msgs::PointField::UINT32, 4},
      {sensor_msgs::PointField::FLOAT32, 4},
      {sensor_msgs::PointField::FLOAT64, 8},
  };
  auto it = kDataTypeSizeMap.find(datatype);
  if (it != kDataTypeSizeMap.end()) {
    return it->second;
  }
  return -1;
}

inline uint8_t *_PackFloat(uint8_t *p, float value) {
  memcpy(p, &value, sizeof(float));
  return p + sizeof(float);
}

inline uint8_t *_PackByte(uint8_t *p, uint8_t value) {
  memcpy(p, &value, sizeof(uint8_t));
  return p + sizeof(uint8_t);
}
}  // namespace

void AppendField(sensor_msgs::PointCloud2 *ros_msg, const char *name,
                 uint8_t datatype) {
  sensor_msgs::PointField field;
  field.name = name;
  field.datatype = datatype;
  field.count = 1;
  if (ros_msg->fields.empty()) {
    field.offset = 0;
  } else {
    field.offset = ros_msg->fields.back().offset +
                   GetSizeForDataType(ros_msg->fields.back().datatype);
  }
  ros_msg->fields.push_back(field);
  ros_msg->point_step += GetSizeForDataType(datatype);
}

sensor_msgs::PointCloud2Ptr ScansToPoints(waymo::Scans *scans,
                                          const hc::HcNodeConfig &config) {
  sensor_msgs::PointCloud2Ptr ros_msg(new sensor_msgs::PointCloud2());
  ros_msg->header.frame_id = config.frame_id;
  ros_msg->is_dense = true;
  ros_msg->is_bigendian = waymo::kIsBigEndianHost;
  ros_msg->height = 1;
  ros_msg->row_step = 0;
  ros_msg->width = 0;
  if (scans->NumScans() == 0) {
    return ros_msg;
  }

  AppendField(ros_msg.get(), "x", sensor_msgs::PointField::FLOAT32);
  AppendField(ros_msg.get(), "y", sensor_msgs::PointField::FLOAT32);
  AppendField(ros_msg.get(), "z", sensor_msgs::PointField::FLOAT32);
  AppendField(ros_msg.get(), "intensity", sensor_msgs::PointField::FLOAT32);
  if (config.publish_spherical_coords) {
    AppendField(ros_msg.get(), "yaw", sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "pitch", sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "range", sensor_msgs::PointField::FLOAT32);
  }
  if (config.publish_raw_intensity) {
    AppendField(ros_msg.get(), "raw_intensity",
                sensor_msgs::PointField::FLOAT32);
  }
  if (config.publish_pulse_width) {
    AppendField(ros_msg.get(), "pulse_width", sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "elongation", sensor_msgs::PointField::FLOAT32);
  }
  if (config.compute_normals) {
    AppendField(ros_msg.get(), "normal_x", sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "normal_y", sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "normal_z", sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "planarity", sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "cos_angle_incidence",
                sensor_msgs::PointField::FLOAT32);
    AppendField(ros_msg.get(), "reflectivity",
                sensor_msgs::PointField::FLOAT32);
  }
  if (config.publish_beam_side) {
    AppendField(ros_msg.get(), "beam_side", sensor_msgs::PointField::UINT8);
  }
  if (config.publish_return_index) {
    AppendField(ros_msg.get(), "return_index", sensor_msgs::PointField::UINT8);
  }
  if (config.publish_return_state) {
    AppendField(ros_msg.get(), "mixed", sensor_msgs::PointField::UINT8);
    AppendField(ros_msg.get(), "noise", sensor_msgs::PointField::UINT8);
    AppendField(ros_msg.get(), "error", sensor_msgs::PointField::UINT8);
  }

  // Allocate space for the data and convert shots to ROS format
  ros_msg->data.resize(scans->NumScans() * scans->MaxShotsPerScan() *
                       scans->MaxReturnsPerShot() * ros_msg->point_step);
  waymo::ReturnMode return_mode =
      static_cast<waymo::ReturnMode>(config.return_mode);

  uint8_t *p = &ros_msg->data[0];
  int num_shots = 0;
  for (const auto &scan : scans->GetScans()) {
    for (const auto &shot : scan) {
      const auto &angle = shot.GetBeamAnglesInDegrees();
      for (const auto &r : shot.GetReturns(return_mode)) {
        ++num_shots;
        const auto &coords = r.GetCoordinates();
        p = _PackFloat(p, coords.x);
        p = _PackFloat(p, coords.y);
        p = _PackFloat(p, coords.z);
        p = _PackFloat(p, r.Intensity());
        if (config.publish_spherical_coords) {
          p = _PackFloat(p, angle.yaw_deg);
          p = _PackFloat(p, angle.pitch_deg);
          p = _PackFloat(p, r.RangeInMeters());
        }
        if (config.publish_raw_intensity) {
          p = _PackFloat(p, r.RawIntensity());
        }
        if (config.publish_pulse_width) {
          p = _PackFloat(p, r.PulseWidthInMeters());
          p = _PackFloat(p, r.ElongationInMeters());
        }
        if (config.compute_normals) {
          const auto &n = r.Normal();
          p = _PackFloat(p, n.x);
          p = _PackFloat(p, n.y);
          p = _PackFloat(p, n.z);
          p = _PackFloat(p, r.Planarity());
          p = _PackFloat(p, r.CosAngleIncidence());
          p = _PackFloat(p, r.Reflectivity());
        }
        if (config.publish_beam_side) {
          p = _PackByte(p, shot.BeamSide());
        }
        if (config.publish_return_index) {
          p = _PackByte(p, r.Index());
        }
        if (config.publish_return_state) {
          p = _PackByte(p, r.IsMixed());
          p = _PackByte(p, r.IsNoise());
          p = _PackByte(p, scan.IsError());
        }
      }
    }
  }

  ros_msg->width = num_shots;
  ros_msg->row_step = ros_msg->point_step * ros_msg->width;
  ros_msg->data.resize(ros_msg->row_step);

  // we use the meantime of start and end of scans as point cloud timestamp
  // as long as there is no motion compensation for translation (de-skewing)
  const auto &start = scans->GetScan(0).Timestamp();
  const auto &end = scans->GetScan(scans->NumScans() - 1).Timestamp();
  ros_msg->header.stamp.fromSec(start + (end - start) / 2);
  return ros_msg;
}

visualization_msgs::MarkerPtr ScansToMarkers(waymo::Scans *scans,
                                             std::string frame_id) {
  visualization_msgs::MarkerPtr ros_msg(new visualization_msgs::Marker());
  static uint32_t counter = 0;
  constexpr int kAccumulation = 4;      // Accumulate 4 scans worth of normals
  constexpr double kLineWidth = 0.005;  // Width of normal lines in meters
  constexpr double kLineLength = 0.05;  // Length of normal lines in meters
  constexpr double kRatio =
      0.4;  // Only publish half of the normals to reduce "crowding"

  ros_msg->header.frame_id = frame_id;
  ros_msg->ns = "normals";
  ros_msg->action = visualization_msgs::Marker::ADD;
  ros_msg->pose.orientation.w = 1.0;
  ros_msg->id = counter++ % kAccumulation;
  ros_msg->type = visualization_msgs::Marker::LINE_LIST;
  ros_msg->scale.x = kLineWidth;
  ros_msg->color.r = ros_msg->color.g = ros_msg->color.b = ros_msg->color.a =
      1.0;

  double w = 0;
  for (const auto &shot : scans->GetShots()) {
    for (const auto &r : shot) {
      const auto &coords = r.GetCoordinates();
      const auto &n = r.Normal();
      if (n.x == 0 && n.y == 0 && n.z == 0) {
        continue;
      }
      w += kRatio;
      if (w < 1) {
        continue;
      }
      w -= 1;
      geometry_msgs::Point p;
      p.x = coords.x;
      p.y = coords.y;
      p.z = coords.z;

      ros_msg->points.push_back(p);
      p.x += n.x * kLineLength;
      p.y += n.y * kLineLength;
      p.z += n.z * kLineLength;
      ros_msg->points.push_back(p);
    }
  }

  const auto &start = scans->GetScan(0).Timestamp();
  const auto &end = scans->GetScan(scans->NumScans() - 1).Timestamp();
  ros_msg->header.stamp.fromSec(start + (end - start) / 2);

  return ros_msg;
}
}  // namespace honeycomb
