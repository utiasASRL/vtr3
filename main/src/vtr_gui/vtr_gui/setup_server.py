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
import time
import os
import flask
import flask_socketio

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


@socketio.on('command/confirm_setup')
def handle_confirm_setup(data):
  logger.info('Received setup parameters', data)
  if vtr_setup.confirm_setup(data):
    logger.info('Broadcasting setup complete')
    socketio.emit(u"notification/setup_complete")
  else:
    logger.info('Broadcasting setup invalid')
    socketio.emit(u"notification/setup_invalid")


@socketio.on('command/request_default_dir')
def handle_default_dir():
  data = vtr_setup.get_default_dir()
  logger.info('Broadcasting default dir')
  socketio.emit(u"notification/default_dir", data)


@socketio.on('command/request_available_subdirs')
def handle_available_subdirs():
  data = vtr_setup.get_available_subdirs()
  logger.info('Broadcasting available subdirs')
  socketio.emit(u"notification/available_subdirs", data)


@socketio.on('command/kill_setup_server')
def handle_kill_setup_server():
  os._exit(0)


def main():
  logger.info("Launching the setup server.")
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)


if __name__ == '__main__':
  main()