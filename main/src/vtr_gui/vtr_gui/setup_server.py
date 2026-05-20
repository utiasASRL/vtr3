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

import logging
import os
import flask
import flask_socketio
from flask_socketio import join_room, leave_room
import argparse
import json


import vtr_navigation.vtr_setup as vtr_setup


# socket io server address and port
SOCKET_ADDRESS = '0.0.0.0'
SOCKET_PORT = 5202

logger = logging.getLogger('SetupServer')
logger.setLevel(logging.INFO)
hd = logging.StreamHandler()
fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
hd.setFormatter(fm)
logger.addHandler(hd)

app = flask.Flask(__name__)
app.config['DEBUG'] = False
app.secret_key = 'asecretekey'

app.logger.setLevel(logging.ERROR)
logging.getLogger('werkzeug').setLevel(logging.ERROR)

socketio = flask_socketio.SocketIO(app,
                                   logger=False,
                                   engineio_logger=False,
                                   ping_interval=1,
                                   ping_timeout=2,
                                   cors_allowed_origins="*")

active_robots = []
allowed_robots = []
all_robot_files = {}

@app.route('/')
def main():
  return "This is a socket-only API server."


@socketio.on('connect')
def on_connect():
  logger.info('Client connected!')


@socketio.on('disconnect')
def on_disconnect():
  logger.info('Client disconnected!')


##### Setup specific calls #####

@socketio.on('join')
def on_join(data):
    robot_name = data['namespace']
    if len(allowed_robots) == 0 or (len(allowed_robots) > 0 and robot_name in allowed_robots):
      active_robots.append(robot_name)
      join_room("robot")
      logger.info("Robot " + robot_name + " is connected.")
      flask_socketio.send('command/request_available_subdirs', to="robot")
    else:
      logger.warning(f"Robot {robot_name} tried to join room but was not in whitelist. Skipped.")


@socketio.on('leave')
def on_leave(data):
    robot_name = data['namespace']
    active_robots.remove(robot_name)
    leave_room("robot")
    logger.info("Robot + " + robot_name + " has disconnected.")

@socketio.on('command/confirm_setup')
def handle_confirm_setup(data):
  logger.info('Received setup parameters', data)
  if vtr_setup.confirm_setup(data):
    logger.info('Broadcasting setup complete')
    socketio.emit(u"notification/setup_complete")
    socketio.emit(u"command/confirm_setup_transfer", data)
  else:
    logger.info('Broadcasting setup invalid')
    socketio.emit(u"notification/setup_invalid")


@socketio.on('command/request_default_dir')
def handle_default_dir():
  data = vtr_setup.get_default_dir()
  logger.info('Broadcasting default dir')
  socketio.emit(u"notification/default_dir", data)

@socketio.on('notification/robot_data')
def handle_robot_files(data):
  all_robot_files[data['namespace']] = (data['times'], data['files'])

  if (len(all_robot_files.keys()) == num_robots):
    common_files = set(list(all_robot_files.values())[0][1])
    print(common_files)
    for _, robot_files in all_robot_files.values():
      print()
      print(robot_files)
      common_files.intersection_update(set(robot_files))

    print(common_files)

    common_elements = {fname: 0 for fname in common_files}

    for robot_times, robot_files in all_robot_files.values():
      for t_i, file_i in zip(robot_times, robot_files):
        if file_i in common_files and t_i > common_elements[file_i]:
          common_elements[file_i] = t_i

    logger.info('Broadcasting available subdirs')

    socketio.emit(u"notification/available_subdirs", sorted(common_elements, key=common_elements.get, reverse=True))


@socketio.on('command/request_available_subdirs')
def handle_available_subdirs():
  logger.info("Passing on request")
  socketio.emit('command/request_available_subdirs')


@socketio.on('command/kill_setup_server')
def handle_kill_setup_server():
  socketio.emit(u"command/kill_setup_client")
  os._exit(0)


def main():
  logger.info("Launching the setup server.")
  parser = argparse.ArgumentParser(description="Setup Server")
  parser.add_argument('--robots', type=str, default="[]", help='Robots allowed to join the room')
  args, unknown = parser.parse_known_args()
  global allowed_robots
  allowed_robots = json.loads(args.robots)
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)



if __name__ == '__main__':
  main()