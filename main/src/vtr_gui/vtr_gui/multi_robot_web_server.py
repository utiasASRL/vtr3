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

import io
import json
import os
import os.path as osp
import logging
import flask
import requests
from requests.exceptions import RequestException
from PIL import Image

from vtr_navigation.multi_robot_vtr_ui_builder import build_remote

## Config the web server
# web server address and port
GUI_ADDRESS = '0.0.0.0'
GUI_PORT = 5200

logger = logging.getLogger('WebServer')
logger.setLevel(logging.INFO)
hd = logging.StreamHandler()
fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
hd.setFormatter(fm)
logger.addHandler(hd)

app = flask.Flask(__name__, static_folder="vtr-gui/build", template_folder="vtr-gui/build", static_url_path="")
app.config['DEBUG'] = False
app.config['CACHE'] = True
app.config['CACHE_PATH'] = osp.abspath(osp.join(osp.dirname(__file__), 'cache'))
app.secret_key = 'asecretekey'

app.logger.setLevel(logging.ERROR)
logging.getLogger('werkzeug').setLevel(logging.ERROR)


@app.route("/")
def main_page():
  return flask.redirect("index.html")

#@app.route("/index.html")
#def load_main_page():
#  return flask.render_template("index.html", multi_robot=True, robots=json.dumps(build_remote().get_robot_namespaces()))

@app.after_request
def set_response_headers(response):
  response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
  response.headers['Pragma'] = 'no-cache'
  response.headers['Expires'] = '0'
  return response


@app.route('/tile/<s>/<x>/<y>/<z>')
def get_tile(s, x, y, z):
  """Load google satellite map"""
  fname = x + '.jpg'
  fdir = osp.join(app.config['CACHE_PATH'], 'tile', z, y)
  fpath = osp.join(fdir, fname)

  if app.config['CACHE'] and osp.isfile(fpath):
    logger.debug(f"Using cached tile {x},{y},{z}")
    return flask.send_from_directory(fdir, fname, max_age=60 * 60 * 24 * 30)

  headers = {'Accept': 'image/webp,image/*,*/*;q=0.8', 'User-Agent': flask.request.user_agent.string}
  # Google Map service
  url = f'http://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}'
  # Open Street Map (mapnik) service
  # url = f'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'

  try:
    res = requests.get(url, headers=headers, verify=False)
  except RequestException as e:
    logger.error(f'Error loading tile {x},{y},{z}: {e}')
    flask.abort(500)

  if not res.ok:
    logger.error(f"Tile {x},{y},{z} did not exist on server")
    flask.abort(404)

  try:
    sio = io.BytesIO(res.content)
    if app.config['CACHE']:
      logger.debug(f"Caching tile: {x},{y},{z}")
      os.makedirs(fdir, exist_ok=True)
      img = Image.open(sio)
      img.save(fpath)
    else:
      logger.debug(f"Proxying tile: {x},{y},{z}")

    sio.seek(0)
  except Exception as e:
    logger.error(f'Something went really sideways on tile {x},{y},{z}: {e}')
    flask.abort(500)
  else:
    return flask.send_file(sio, mimetype='image/jpeg', max_age=60 * 60 * 24 * 30)


##### VTR specific calls #####
@app.route('/vtr/graph/')
def get_graph_state():
    graph_state = build_remote().get_graph_state()
    return flask.jsonify(graph_state)

@app.route('/vtr/additional_data')
def get_additional_data():
  # Currently just robot IDs, but could be extended
  robots = build_remote().get_robot_ids()
  return flask.jsonify(robots)



@app.route('/vtr/robot/<robot_id>')
def get_robot_state(robot_id):
  robot_state = build_remote().get_robot_state(robot_id)
  return flask.jsonify(robot_state)


@app.route('/vtr/following_route/<robot_id>')
def get_following_route(robot_id):
  following_route = build_remote().get_following_route(robot_id)
  return flask.jsonify(following_route)


@app.route('/vtr/server/<robot_id>')
def get_server_state(robot_id):
  server_state = build_remote().get_server_state(robot_id)
  print(f"Web Server: Server state for robot {robot_id}: {server_state}") 
  return flask.jsonify(server_state)


@app.route('/vtr/task_queue/')
def get_task_queue_state():
    task_queue_state = build_remote().get_task_queue_state()
    return flask.jsonify(task_queue_state)

@app.route('/vtr/map_info')
def get_map_info():
  map_info = build_remote().get_map_info()
  return flask.jsonify(map_info)

def main():
  logger.info("Launching the web server.")
  app.run(threaded=True, host=GUI_ADDRESS, port=GUI_PORT, use_reloader=False)


if __name__ == '__main__':
  main()
