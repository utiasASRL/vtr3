#!/usr/bin/env python
"""
import re
import requests
from requests.packages.urllib3.exceptions import InsecurePlatformWarning, SNIMissingWarning
from requests.exceptions import (ConnectTimeout, ReadTimeout, ConnectionError,
                                 HTTPError, RequestException)
import shutil
from flask import (Flask, redirect, jsonify, render_template,
                   send_from_directory, request, make_response, abort,
                   send_file)
import logging
import json
import io
from io import open
import numpy as np
from functools import wraps

from PIL import Image

from . import utils
from vtr_planning.msg import MissionStatus

from vtr_planning import remote_client, Notification

from asrl__pose_graph.msg import GraphComponent
from proto.Graph_pb2 import Graph, GraphOverlay
"""
import os
import logging
osp = os.path

import flask

from vtr_interface import UI_ADDRESS, UI_PORT

UI_ADDRESS = "0.0.0.0"
UI_PORT = 5201

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


if __name__ == '__main__':
  log.info("Launching UI server.")

  # TODO: Server runs on all interfaces.  Can we assume a trusted network?
  app.run(threaded=True, host=UI_ADDRESS, port=UI_PORT, use_reloader=False)
