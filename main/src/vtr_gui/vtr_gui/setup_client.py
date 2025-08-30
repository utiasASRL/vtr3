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

import vtr_navigation.vtr_setup as vtr_setup
import socketio


# TODO: Make this configurable


logger = logging.getLogger('setupclient')

logger.setLevel(logging.INFO)
hd = logging.StreamHandler()
fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
hd.setFormatter(fm)
logger.addHandler(hd)

class SetupClient:
  def __init__(self, SOCKET_ADDRESS='0.0.0.0', SOCKET_PORT=5202):
    self.SOCKET_ADDRESS = SOCKET_ADDRESS
    self.SOCKET_PORT = SOCKET_PORT  
    self.shutdown = False
    self._socketio = socketio.Client(logger=True, engineio_logger=True)
  

  def on_connect(self,):
    logger.info('Client connected!')

  def on_disconnect(self, reason=None, ):
    logger.info('Client disconnected!')
    os._exit(0)

  ##### Setup specific calls #####

  def handle_confirm_setup(self, data):
    logger.info('Received setup parameters', data)
    # TEMP for local data
    data['data_dir'] += "_2"
    if vtr_setup.confirm_setup(data):
      logger.info('Setup complete')
      self.shutdown = True
    else:
      logger.info('Setup invalid')


  def handle_kill_setup_client(self, ):
    self.shutdown = True
    logger.info('Shutting down setup client')
    os._exit(0)


  def run(self, ):
    logger.info("Launching the setup client.")

    while True:
      try:
        self._socketio.connect('http://' + self.SOCKET_ADDRESS + ':' + str(self.SOCKET_PORT))
        break
      except socketio.exceptions.ConnectionError:
        vtr_ui_logger.info("Waiting for socket io server...")
        time.sleep(1)
    self._socketio.on('connect', self.on_connect)
    self._socketio.on('disconnect', self.on_disconnect)
    self._socketio.on('command/confirm_setup_transfer', self.handle_confirm_setup)
    self._socketio.on('command/kill_setup_client', self.handle_kill_setup_client)
    while True:
      self._socketio.wait()
      if self.shutdown:
        self._socketio.disconnect()
        break
    

def main():
  setup_client = SetupClient()
  setup_client.run()  

if __name__ == '__main__':
  setup_client = SetupClient()
  setup_client.run()