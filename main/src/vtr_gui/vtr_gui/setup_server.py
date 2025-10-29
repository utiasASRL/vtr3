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

robots = []
num_robots = 1
robot_files = {}

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
    robots.append(robot_name)
    join_room("robot")
    logger.info("Robot " + robot_name + " is connected.")
    flask_socketio.send('command/request_available_subdirs', to="robot")


@socketio.on('leave')
def on_leave(data):
    robot_name = data['namespace']
    robots.remove(robot_name)
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
  robot_files[data['namespace']] = set(data['files'])
  if (len(robot_files.keys()) == num_robots):
    common_elements = None
    # Iterate through the rest of the sets and apply the intersection operator
    for s in robot_files.values():
      if common_elements is None:
        common_elements = s
      else:
        common_elements = common_elements & s
    logger.info('Broadcasting available subdirs')
    socketio.emit(u"notification/available_subdirs", list(common_elements))


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
  parser.add_argument('--num-robots', type=int, default=1, help='Number of robots to expect')
  args, unknown = parser.parse_known_args()
  global num_robots
  num_robots = args.num_robots
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)



if __name__ == '__main__':
  main()