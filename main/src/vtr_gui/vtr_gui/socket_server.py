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
import flask
import flask_socketio

from vtr_navigation_v2.vtr_ui_builder import build_remote

# socket io server address and port
SOCKET_ADDRESS = 'localhost'
SOCKET_PORT = 5201

logger = logging.getLogger('SocketServer')
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


##### VTR specific calls #####


@socketio.on('command/annotate_route')
def handle_annotate_route(data):
  logger.info('Received annotate route command', data)


@socketio.on('command/move_graph')
def handle_move_graph(data):
  logger.info('Received move graph command', data)
  build_remote().move_graph(data)


@socketio.on('notification/graph_state')
def handle_graph_state(json):
  logger.info('Broadcasting graph state')
  graph_state = json['graph_state']
  socketio.emit(u"graph/state", graph_state, broadcast=True)


def main():
  logger.info("Launching the socket server.")
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)


if __name__ == '__main__':
  main()