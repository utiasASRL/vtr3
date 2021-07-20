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

#include <unistd.h>

#include <iostream>

#include "honeycomb/honeycomb.h"

int main(int argc, char** argv) {
  waymo::Status status;
  waymo::Honeycomb lidar;

  if ((status = lidar.Init()) != waymo::Status::kOk) {
    std::cerr << "Failed to initialize Honeycomb lidar: " << status
              << std::endl;
    return 1;
  }

  if ((status = lidar.Run()) != waymo::Status::kOk) {
    std::cerr << "Failed to run Honeycomb lidar: " << status << std::endl;
    return 1;
  }

  if (lidar.SetSpinFrequency(10.0) != waymo::Status::kOk ||
      lidar.SetFieldOfView(210, 0) != waymo::Status::kOk ||
      lidar.SetSides(waymo::Sides::kBothSides) != waymo::Status::kOk ||
      lidar.SetComputeNormals(waymo::Sides::kNone) != waymo::Status::kOk) {
    std::cerr << "Failed to set Honeycomb options." << std::endl;
    return 1;
  }

  std::cout << "Waiting for steady state" << std::endl;
  while (!lidar.IsSteadyState()) {
    sleep(1);  // wait and poll after 1 second
  }
  std::cout << "In steady state" << std::endl;

  int shard_num = 0;
  std::cout << "x,y,z,intensity" << std::endl;
  while (lidar.IsRunning() && shard_num < 10) {
    waymo::ScansPtr scans;
    if ((status = lidar.GetLidarData(waymo::kDurationForever, true, &scans)) !=
        waymo::Status::kOk) {
      std::cerr << "Failed to get lidar data: " << status << std::endl;
      return 1;
    }
    for (const waymo::Shot& shot : scans->GetShots()) {
      for (const waymo::Return& ret : shot) {
        const auto& coords = ret.GetCoordinates();
        std::cout << coords.x << "," << coords.y << "," << coords.z << ","
                  << ret.Intensity() << std::endl;
      }
    }
    shard_num++;
  }
  if ((status = lidar.Stop()) != waymo::Status::kOk) {
    std::cerr << "Failed to stop Honeycomb lidar: " << status << std::endl;
    return 1;
  }

  return 0;
}
