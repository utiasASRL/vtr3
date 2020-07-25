#!/usr/bin/env python

import os
import logging
osp = os.path

import numpy as np
import flask

from vtr_interface import UI_ADDRESS, UI_PORT
from vtr_interface import utils
from vtr_interface.proto import Graph_pb2

from asrl__pose_graph.msg import GraphComponent

logging.basicConfig(level=logging.WARNING)

log = logging.getLogger('WebServer')
log.setLevel(logging.INFO)

# Web server config
app = flask.Flask(__name__,
                  static_folder="../../vtr_frontend/vtr-ui/build",
                  template_folder="../../vtr_frontend/vtr-ui/build",
                  static_url_path="")
app.config['DEBUG'] = True
app.secret_key = 'asecretekey'

app.logger.setLevel(logging.ERROR)
logging.getLogger('werkzeug').setLevel(logging.ERROR)


@app.route("/")
def main_page():
  return flask.redirect("index.html")


@app.route('/api/map/<seq>')
def get_map(seq):
  """API endpoint to get the full map"""
  graph = utils.get_graph(seq)

  proto_graph = Graph_pb2.Graph()
  proto_graph.seq = graph.seq
  proto_graph.stamp = graph.stamp.to_sec()
  proto_graph.root = graph.rootId

  if graph.seq < 0:
    return proto_graph.SerializeToString()

  if not graph.projected:
    raise RuntimeError(
        "Received an unprojected graph... What do I do with this?")

  for v in graph.vertices:
    vertex = proto_graph.vertices.add()
    vertex.id = v.id
    vertex.lat = v.T_projected.y
    vertex.lng = v.T_projected.x
    vertex.theta = v.T_projected.theta
    vertex.neighbours.extend(v.neighbours)

  for c in graph.components:
    if c.type == GraphComponent.PATH:
      component = proto_graph.paths.add()
    elif c.type == GraphComponent.CYCLE:
      component = proto_graph.cycles.add()
    else:
      raise RuntimeError(
          "Encountered unknown graph component type in UI Server")

    component.vertices.extend(c.vertices)

  proto_graph.branch.vertices.extend(graph.active_branch)
  proto_graph.junctions.extend(graph.junctions)

  gps_coords = np.array(
      [[v.T_projected.y, v.T_projected.x] for v in graph.vertices])

  if len(gps_coords > 0):
    mn = list(np.min(np.array(gps_coords), axis=0))
    mx = list(np.max(np.array(gps_coords), axis=0))
  elif len(graph.center) == 2:
    mn = graph.center
    mx = graph.center
  else:
    mn = [43.781596, -79.467298]
    mx = [43.782806, -79.464608]

  proto_graph.min_bnd.lat = mn[0]
  proto_graph.min_bnd.lng = mn[1]
  proto_graph.max_bnd.lat = mx[0]
  proto_graph.max_bnd.lng = mx[1]
  proto_graph.map_center.lat = (mn[0] + mx[0]) / 2
  proto_graph.map_center.lng = (mn[1] + mx[1]) / 2

  return proto_graph.SerializeToString()


if __name__ == '__main__':
  log.info("Launching UI server.")

  # TODO: Server runs on all interfaces.  Can we assume a trusted network?
  app.run(threaded=True, host=UI_ADDRESS, port=UI_PORT, use_reloader=False)
