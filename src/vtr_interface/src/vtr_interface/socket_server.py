#!/usr/bin/env python

import logging
import flask
import flask_socketio

from vtr_interface import SOCKET_ADDRESS, SOCKET_PORT

logging.basicConfig(level=logging.WARNING)

log = logging.getLogger('SocketServer')
log.setLevel(logging.INFO)

# Web server config
app = flask.Flask(__name__)
app.config['DEBUG'] = True
app.secret_key = 'asecretekey'

elog = logging.getLogger('engineio')
slog = logging.getLogger('socketio')
rlog = logging.getLogger('requests')

elog.setLevel(logging.ERROR)
slog.setLevel(logging.ERROR)
rlog.setLevel(logging.ERROR)

logging.getLogger('werkzeug').setLevel(logging.ERROR)
app.logger.setLevel(logging.ERROR)

socketio = flask_socketio.SocketIO(app,
                                   logger=slog,
                                   engineio_logger=elog,
                                   ping_interval=1,
                                   ping_timeout=2,
                                   cors_allowed_origins="*")


@socketio.on('connect')
def on_connect():
  log.info('Client connected!')
  # TODO move to the correct function.
  socketio.emit("robot/loc", "16")


@socketio.on('disconnect')
def on_disconnect():
  log.info('Client disconnected!')


@app.route('/')
def main():
  return "This is a socket-only API server."


if __name__ == '__main__':
  log.info("Launching socket server...")

  # TODO: Server runs on all interfaces.  Can we assume a trusted network?
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)