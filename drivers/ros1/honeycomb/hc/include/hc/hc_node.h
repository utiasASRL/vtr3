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
#ifndef HC_NODE_H_
#define HC_NODE_H_

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <condition_variable>  // NOLINT
#include <memory>
#include <mutex>  // NOLINT
#include <thread>  // NOLINT

#include "honeycomb/honeycomb.h"
#include "hc/HcNodeConfig.h"
#include "hc/scan_image_producer.h"


namespace honeycomb {

class HcRosNode {
 public:
  bool Init();
  waymo::Status Run();

  ~HcRosNode();

  void UpdateStatistics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void UpdateStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void UpdateSystemErrors(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void UpdateSystemInfo(diagnostic_updater::DiagnosticStatusWrapper &stat);

 private:
  std::unique_ptr<waymo::Honeycomb> lidar_;
  waymo::Honeycomb::SystemInfo system_info_;
  void dynamic_reconfigureCB(const hc::HcNodeConfig &config, uint32_t level);

  std::thread points_publisher_thread_;
  std::thread imu_publisher_thread_;

  ::ros::Publisher points_publisher_;
  ::ros::Publisher normals_publisher_;
  ::ros::Publisher imu_publisher_;
  ::ros::Publisher range_image_publisher_;
  ::ros::Publisher intensity_image_publisher_;
  ::ros::NodeHandle pnh_;

  // dynamic_reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<hc::HcNodeConfig>>
      dynamic_reconfigure_server_;

  std::mutex config_mtx_;
  std::condition_variable config_cv_;
  hc::HcNodeConfig config_;

  ScanImageProducer scan_image_producer_;

  void GetCurrentPitchTable();
  std::vector<double> current_pitch_table_;
  waymo::PitchTableLimits pitch_table_limits_;

  bool is_interlaced_;

  void GeneratePitchTables();
  void PublishPoints();
  void PublishImu();
  std::map<int, std::vector<double>> pitch_tables_;
};
}  // namespace honeycomb

#endif  // HC_NODE_H_
