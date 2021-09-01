#!/usr/bin/env python3

# Copyright 2021, Autonomous Space Robotics Lab (ASRL)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from vtr_mission_planning.mission_client_builder import build_master_client

# Do not remove this if statement; it will cause fork bombs
if __name__ == '__main__':
  client, mgr = build_master_client()

  # This has to happen in a specific order:

  # 1) Start the mission client, spawning a separate process with ROS in it
  client.start()

  # 2) Start a python Multiprocessing.Manager that contains the client.  This blocks the main process.
  mgr.get_server().serve_forever()

  # 3) The server is interrupted (Ctrl-C), which shuts down the Multiprocessing.Manager
  # 4) Shut down the client, stopping ROS and joining the process that was spawned in (1)
  client.shutdown()
