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

from vtr_navigation.vtr_ui_builder import build_remote

# socket io server address and port
SOCKET_ADDRESS = '0.0.0.0'
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


@socketio.on('command/set_pause')
def handle_set_pause(data):
  logger.info('Received add goal command', data)
  build_remote().set_pause(data)


@socketio.on('command/add_goal')
def handle_add_goal(data):
  logger.info('Received add goal command', data)
  build_remote().add_goal(data)


@socketio.on('command/cancel_goal')
def handle_cancel_goal(data):
  logger.info('Received cancel goal command', data)
  build_remote().cancel_goal(data)


@socketio.on('command/merge')
def handle_merge(data):
  logger.info('Received merge command', data)
  build_remote().merge(data)


@socketio.on('command/confirm_merge')
def handle_confirm_merge(data):
  logger.info('Received confirm merge command', data)
  build_remote().confirm_merge(data)


@socketio.on('command/continue_teach')
def handle_continue_teach(data):
  logger.info('Received continue teach command', data)
  build_remote().continue_teach(data)


@socketio.on('command/annotate_route')
def handle_annotate_route(data):
  logger.info('Received annotate route command', data)
  build_remote().annotate_route(data)


@socketio.on('command/move_graph')
def handle_move_graph(data):
  logger.info('Received move graph command', data)
  build_remote().move_graph(data)


@socketio.on('command/move_robot')
def handle_move_robot(data):
  logger.info('Received move robot command', data)
  build_remote().move_robot(data)


@socketio.on('command/change_env_info')
def handle_change_env_info(data):
  logger.info('Received change env info command', data)
  build_remote().change_env_info(data)


@socketio.on('notification/server_state')
def handle_server_state(json):
  logger.info('Broadcasting server state')
  server_state = json['server_state']
  socketio.emit(u"mission/server_state", server_state)


@socketio.on('notification/graph_state')
def handle_graph_state(json):
  logger.info('Broadcasting graph state')
  graph_state = json['graph_state']
  socketio.emit(u"graph/state", graph_state)


@socketio.on('notification/graph_update')
def handle_graph_update(json):
  logger.info('Broadcasting graph update')
  graph_update = json['graph_update']
  socketio.emit(u"graph/update", graph_update)


@socketio.on('notification/robot_state')
def handle_robot_state(json):
  logger.info('Broadcasting robot state')
  robot_state = json['robot_state']
  socketio.emit(u"robot/state", robot_state)


@socketio.on('notification/following_route')
def handle_following_route(json):
  logger.info('Broadcasting following route')
  following_route = json['following_route']
  socketio.emit(u"following_route", following_route)


@socketio.on('notification/task_queue_update')
def handle_task_queue_update(json):
  logger.info('Broadcasting task queue update')
  task_queue_update = json['task_queue_update']
  socketio.emit(u"task_queue/update", task_queue_update)


def main():
  logger.info("Launching the socket server.")
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)


if __name__ == '__main__':
  main()