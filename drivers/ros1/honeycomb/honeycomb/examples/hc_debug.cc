/*
 * Unpublished Work Copyright 2021 Waymo LLC.  All rights reserved.
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

#include <signal.h>
#include <unistd.h>

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include "honeycomb/honeycomb.h"

// whether interrupt is received requesting to stop
volatile int stop_requested = 0;

void IntHandler(int signum) {
  std::cerr << "Stop requested " << std::endl;
  stop_requested = 1;
}

void PrintUsage() {
  std::cout << R"(Usage: hc_debug [options]
Options:
  -h    Help, print this message and exit
  -i    Set IP address
  -p    Select sensor using MAC address
  -p    Print points (x,y,z,intensity)
  -l    Loop forever otherwise it runs for 10 shards
  -e    Show system errors
  -t    Show timestamp and sequence no.
  -s    Show system stats every 10th shard
  -r    Restart (stop and run) every 10th shard
  -w    Wait for shard forever, otherwise timeout in 0.5 sec)"
            << std::endl;
}

void PrintLidarSystemError(waymo::Honeycomb& lidar) {
  std::vector<waymo::Honeycomb::SystemError> system_errors;
  if (lidar.GetSystemErrors(&system_errors) == waymo::Status::kOk) {
    for (const auto& error : system_errors) {
      std::cout << error.severity << ": ";
      if (!error.code.empty()) {
        std::cout << error.code << ": ";
      }
      std::cout << error.description << std::endl;
    }
  }
}

void PrintLidarTimestampSeqNo(waymo::Honeycomb& lidar) {
  waymo::Honeycomb::SystemStats system_stats;
  if (lidar.GetStatsData(0, true, &system_stats) == waymo::Status::kOk) {
    std::cout << "Sequence no.: "
              << system_stats.find("sequence_number")->second.GetValue()
              << " timestamp: "
              << system_stats.find("timestamp")->second.GetValue() << std::endl;
  }
}

void PrintLidarStats(waymo::Honeycomb& lidar) {
  waymo::Honeycomb::SystemStats system_stats;
  if (lidar.GetStatsData(0.5, true, &system_stats) == waymo::Status::kOk) {
    std::cout << "SystemStats:\n";
    for (const auto& stat : system_stats) {
      std::cout << "\t" << stat.first << " = " << stat.second.GetValue()
                << std::endl;
    }
    std::cout << std::endl;
  }
}

void PrintLidarInfo(waymo::Honeycomb& lidar) {
  waymo::Honeycomb::SystemInfo system_info;
  if (lidar.GetSystemInfo(&system_info) == waymo::Status::kOk) {
    std::cout << "Honeycomb information:\n"
              << "  IP Address: " << system_info.ip_address << std::endl
              << "  MAC Address: " << system_info.mac_address << std::endl
              << "  Serial Number: " << system_info.serial_number << std::endl
              << "  Model Number: " << system_info.model_number << std::endl
              << "  Software Version: " << system_info.software_build_version
              << std::endl
              << "  Software Build Date: " << system_info.software_build_date
              << std::endl;
  }
}

void WaitForSteadyState(waymo::Honeycomb& lidar, bool show_system_errors) {
  std::cout << "Waiting for steady state" << std::endl;
  while (!stop_requested && !lidar.IsSteadyState()) {
    // System error while waiting if enabled
    if (show_system_errors) {
      PrintLidarSystemError(lidar);
    }
    sleep(1);  // wait and poll after 1 second
  }
  if (lidar.IsSteadyState()) {
    std::cout << "In steady state" << std::endl;
  }
}

int main(int argc, char** argv) {
  waymo::Status status;
  waymo::Honeycomb lidar;
  std::vector<waymo::Honeycomb::SystemError> system_errors;

  bool show_points = false;
  bool loop_forever = false;
  bool show_system_errors = false;
  bool show_time_seq = false;
  bool show_system_stats = false;
  bool do_restarts = false;
  double timeout_sec = 0.5;
  std::string ip_addr;
  std::string mac_address;
  int opt;

  std::ostringstream mode;
  mode << "Mode:\n";

  while ((opt = getopt(argc, argv, "hi:m:pletsrw")) != -1) {
    switch (opt) {
      case 'h':
        PrintUsage();
        return EXIT_SUCCESS;
        break;
      case 'i':
        ip_addr.assign(optarg);
        std::cout << "Setting IP address: " << ip_addr << std::endl;
        break;
      case 'm':
        mac_address.assign(optarg);
        std::cout << "Selecting sensor with MAC address :" << mac_address
                  << std::endl;
        break;
      case 'p':
        mode << "\tPrint points (x,y,z,intensity) \n";
        show_points = true;
        break;
      case 'l':
        mode << "\tLoop forever\n";
        loop_forever = true;
        break;
      case 'e':
        mode << "\tShow system errors\n";
        show_system_errors = true;
        break;
      case 't':
        mode << "\tShow timestamp and sequence no.\n";
        show_time_seq = true;
        break;
      case 's':
        mode << "\tShow system stats every 10th shard\n";
        show_system_stats = true;
        break;
      case 'r':
        mode << "\tRestart (stop and run) every 10th shard\n";
        do_restarts = true;
        break;
      case 'w':
        mode << "\tWait for shard forever\n";
        timeout_sec = waymo::kDurationForever;
        break;
      case '?':
        PrintUsage();
        return EXIT_FAILURE;
        break;
      default:
        std::cout << "\tInvalid option " << opt << std::endl;
    }
  }

  if (!loop_forever) {
    mode << "\tRun for 10 shards and exit \n";
  }
  std::cout << mode.str() << std::endl;

  std::cout << "Lidar Init(" << ip_addr << ", " << mac_address << ")"
            << std::endl;
  if ((status = lidar.Init(ip_addr, mac_address)) != waymo::Status::kOk) {
    std::cerr << "Failed to initialize Honeycomb lidar: " << status
              << std::endl;
    return EXIT_FAILURE;
  }

  PrintLidarInfo(lidar);

  std::cout << "Lidar Run()" << std::endl;
  if ((status = lidar.Run()) != waymo::Status::kOk) {
    std::cerr << "Failed to run Honeycomb lidar: " << status << std::endl;
    if (show_system_errors) {
      PrintLidarSystemError(lidar);
    }
    return EXIT_FAILURE;
  }

  if (show_system_errors) {
    PrintLidarSystemError(lidar);
  }

  signal(SIGINT, IntHandler);

  WaitForSteadyState(lidar, show_system_errors);

  int shard_num = 0;

  while (!stop_requested && lidar.IsRunning() &&
         (loop_forever || shard_num < 10)) {
    // System error every shard if enabled
    if (show_system_errors) {
      PrintLidarSystemError(lidar);
    }

    // Get a shard
    waymo::ScansPtr scans;
    if ((status = lidar.GetLidarData(timeout_sec, true, &scans)) !=
        waymo::Status::kOk) {
      std::cerr << "Failed to get lidar data: " << status << std::endl;
      return EXIT_FAILURE;
    }

    if (show_points) {
      for (const waymo::Shot& shot : scans->GetShots()) {
        for (const waymo::Return& ret : shot) {
          const auto& coords = ret.GetCoordinates();
          std::cout << "X: " << coords.x << ", Y: " << coords.y
                    << ", Z: " << coords.z << ", Intensity: " << ret.Intensity()
                    << std::endl;
        }
      }
    }

    // print timestamp and sequence number
    if (show_time_seq) {
      PrintLidarTimestampSeqNo(lidar);
    }

    ++shard_num;

    // Every 10th shard
    if (shard_num % 10 == 0) {
      // print stats
      if (show_system_stats) {
        PrintLidarStats(lidar);
      }

      // restart if enabled
      if (do_restarts) {
        std::cout << "Stopping for a restart\n";
        if ((status = lidar.Stop()) != waymo::Status::kOk) {
          std::cerr << "Failed to stop Honeycomb lidar: " << status
                    << std::endl;
          return EXIT_FAILURE;
        }
        std::cout << "Starting for a restart\n";
        if ((status = lidar.Run()) != waymo::Status::kOk) {
          std::cerr << "Failed to run Honeycomb lidar: " << status << std::endl;
          return EXIT_FAILURE;
        }
        WaitForSteadyState(lidar, show_system_errors);
      }

      // Print something if nothing else may be printed
      if (!show_time_seq && !show_system_stats && !do_restarts &&
          !show_points) {
        std::cout << "Received " << shard_num << " shards" << std::endl;
      }
    }
  }

  std::cout << "Lidar Stop()" << std::endl;
  if ((status = lidar.Stop()) != waymo::Status::kOk) {
    std::cerr << "Failed to stop Honeycomb lidar: " << status << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
