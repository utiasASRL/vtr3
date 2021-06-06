#!/usr/bin/env python

import logging
import flask
import flask_socketio

import rclpy

from action_msgs.msg import GoalStatus

import vtr_mission_planning
from vtr_interface import SOCKET_ADDRESS, SOCKET_PORT
from vtr_interface import graph_pb2
from vtr_interface import utils
from vtr_messages.action import Mission
from vtr_messages.srv import MissionPause, MissionCmd
from vtr_messages.msg import MissionStatus

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

# ROS2 node
rclpy.init()
node = rclpy.create_node("socket_server")
node.get_logger().info('Created node - socket_server')


@socketio.on('connect')
def on_connect():
  log.info('Client connected!')


@socketio.on('disconnect')
def on_disconnect():
  log.info('Client disconnected!')


@socketio.on('kill')
def kill():
  """Stops the server politely"""
  log.info("Received termination request. Shutting down!")
  socketio.stop()


@socketio.on('goal/add')
def add_goal(json):
  """Handles SocketIO request to add a goal"""
  log.info('Client requests to add a goal!')
  goal_str = json.get('type', None)
  if goal_str is None:
    return False, u"Goal type is a mandatory field"

  goal_type = {
      'IDLE': Mission.Goal.IDLE,
      'TEACH': Mission.Goal.TEACH,
      'REPEAT': Mission.Goal.REPEAT,
      'MERGE': Mission.Goal.MERGE,
      'LOCALIZE': Mission.Goal.LOCALIZE
  }.get(goal_str.upper(), None)

  if goal_type is None:
    return False, u"Invalid goal type"

  try:
    pause_before = float(json.get('pauseBefore', 0.0))
    pause_after = float(json.get('pauseAfter', 0.0))
  except Exception:
    return False, u"Non-numeric pause duration received"

  try:
    log.info(json.get('path', []))
    path = [int(x) for x in json.get('path', [])]
  except Exception:
    return False, u"Non-integer vertex id supplied in path"

  if goal_type == Mission.Goal.REPEAT and path == []:
    return False, u"Empty path supplied for repeat goal"

  rclient = vtr_mission_planning.remote_client()
  goal_id = rclient.add_goal(goal_type, path, pause_before, pause_after,
                             json.get('vertex', 2**64 - 1))
  log.info("New goal: %s", goal_str)

  return True, goal_id


@socketio.on('goal/cancel')
def cancel_goal(json):
  """Handles SocketIO request to cancel a goal"""
  log.info('Client requests to cancel a goal!')
  goal_id = json.get('id', 'all')
  log.info("Cancelling goal %s", goal_id)

  if goal_id == 'all':
    result = vtr_mission_planning.remote_client().cancel_all()
  else:
    result = vtr_mission_planning.remote_client().cancel_goal(goal_id)

  return result


@socketio.on('goal/cancel/all')
def cancel_all():
  """Handles SocketIO request to cancel all goals"""
  log.info('Client requests to cancel all goals!')
  return vtr_mission_planning.remote_client().cancel_all()


@socketio.on('goal/move')
def move_goal(json):
  """Handles SocketIO request to re-arrange goals"""
  log.info('Client requests to re-arrange goals!')


@socketio.on('pause')
def pause(json):
  """Handle socketIO messages to pause the mission server

  :param json: dictionary containing the desired pause state
  """
  log.info('Client requests to pause the mission server!')
  paused = json.get('paused', None)
  if paused is None:
    return False, u"The pause state is a mandatory parameter"

  # result =
  vtr_mission_planning.remote_client().set_pause(paused)

  # if result == MissionPause.FAILURE:
  #   return False, u"Pause change failed"
  # elif result == MissionPause.PENDING:
  #   return True, u"Pause in progress"
  # elif result == MissionPause.SUCCESS:
  #   return True, u"Pause change succeeded"
  # else:
  #   log.warning(
  #       "An unexpected response code (%d) occurred while pausing the mission",
  #       result)
  #   return False, u"An unknown error occurred; check the console for more information"
  return True


@socketio.on('map/offset')
def move_map(json):
  """Handles SocketIO request to update the map offset"""
  log.info('Client requests to update the map offset!')
  utils.update_graph(node, float(json["x"]), float(json["y"]),
                     float(json["theta"]), json["scale"])


@socketio.on('graph/cmd')
def graph_cmd(req):
  """Handles SocketIO request of a graph command"""
  log.info('Client sends a request of graph command!')

  ros_service = node.create_client(MissionCmd, "mission_cmd")
  while not ros_service.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')

  action = {
      'add_run': MissionCmd.Request.ADD_RUN,  # TODO unused?
      'localize': MissionCmd.Request.LOCALIZE,
      'merge': MissionCmd.Request.START_MERGE,
      'closure': MissionCmd.Request.CONFIRM_MERGE,
      'loc_search': MissionCmd.Request.LOC_SEARCH,  # TODO unused?
  }.get(req['action'], None)

  if action is None:
    raise RuntimeError("Invalid request string")

  request = MissionCmd.Request()
  request.action = action
  if 'vertex' in req.keys():
    request.vertex = req['vertex']
  if 'path' in req.keys():
    request.path = req['path']

  response = ros_service.call_async(request)
  rclpy.spin_until_future_complete(node, response)

  return response.result()


@socketio.on('dialog/response')
def dialog_response(json):
  """Handles SocketIO request of a dialog response(TODO what is this?)"""
  log.info('Client sends a dialog response!')


@socketio.on('message')
def handle_notifications(json):
  """Re-broadcasts incoming notifications from the mission client, every message
  from VTR2 goes to here.

  :param json Dictionary with keys 'args' (list/tuple) and 'kwargs' (dict)
  """
  # Match vtr_planning.mission_client.Notification
  callbacks = {
      'NewGoal': broadcast_new,
      'Error': broadcast_error,
      'Cancel': broadcast_cancel,
      'Complete': broadcast_success,
      'Started': broadcast_started,
      'Feedback': broadcast_feedback,
      'StatusChange': broadcast_status,
      'RobotChange': broadcast_robot,
      'PathChange': broadcast_path,
      'GraphChange': broadcast_graph,
      'SafetyStatus': broadcast_safety,
      'NewDialog': broadcast_new_dialog,
      'CancelDialog': broadcast_cancel_dialog,
      'ReplaceDialog': broadcast_replace_dialog,
      'FinishDialog': broadcast_finish_dialog,
      'OverlayRefresh': broadcast_overlay
  }

  try:
    callbacks[json['type']](*json['args'], **json['kwargs'])
  except KeyError as e:
    log.error('A client disconnected ungracefully: {}.'.format(str(e)))


def broadcast_new(goal):
  """Broadcasts socketIO messages to all clients on new goal addition

  :param goal Dictionary representing the fields of the added goal
  """
  log.info('Broadcast new goal: %s.', str(goal))
  socketio.emit(u"goal/new", goal, broadcast=True)


def broadcast_error(goal_id, goal_status):
  """Broadcasts socketIO messages to all clients on errors

  :param goal_id The id of the failed goal
  :param goal_status Status enum representing the failure reason
  """
  log.info('Broadcast error.')
  # if goal_status == GoalStatus.ABORTED:
  #   msg = "An internal error has occurred!"
  # elif goal_status == GoalStatus.LOST:
  #   msg = "Goal was lost; check network connectivity"
  # elif goal_status == GoalStatus.REJECTED:
  #   msg = "Goal was malformed/missing information; please try again"
  # else:
  msg = "An unknown error occurred; check the console for more information"
  log.warning(
      "An unexpected goal status (%d) occurred while broadcasting errors",
      goal_status)

  socketio.emit(u"goal/error", {'id': goal_id, 'msg': msg}, broadcast=True)


def broadcast_cancel(goal_id):
  """Broadcasts socketIO messages to all clients on goal cancellation

  :param goal_id The id of the cancelled goal
  """
  log.info('Broadcast cancel.')
  socketio.emit(u"goal/cancelled", {'id': goal_id}, broadcast=True)


def broadcast_success(goal_id):
  """Broadcasts socketIO messages to all clients on goal completion

  :param goal_id The id of the finished goal
  """
  log.info('Broadcast success.')
  socketio.emit(u"goal/success", {'id': goal_id}, broadcast=True)


def broadcast_started(goal_id):
  """Broadcasts socketIO messages to all clients when a goal becomes active

  :param goal_id The id of the goal that was started
  """
  log.info('Broadcast started.')
  socketio.emit(u"goal/started", {'id': goal_id}, broadcast=True)


def broadcast_feedback(goal_id, feedback):
  """Broadcasts socketIO messages to all clients on new feedback for any goal

  :param goal_id The id of the goal receiving feedback
  :param feedback Dictionary representing the feedback message
  """
  log.info('Broadcast feedback.')
  data = feedback
  data['id'] = goal_id
  socketio.emit(u"goal/feedback", data, broadcast=True)


def broadcast_status(status, queue):
  """Broadcasts socketIO messages to all clients on status change in the mission
  server status

  :param status The current state of the mission server {EMPTY|PAUSED|PROCESSING|PENDING_PAUSE}
  :param queue List of all current goal ids, ordered by execution priority
  """
  log.info('Broadcast status.')
  text = {
      MissionStatus.PROCESSING: "PROCESSING",
      MissionStatus.PAUSED: "PAUSED",
      MissionStatus.PENDING_PAUSE: "PENDING_PAUSE",
      MissionStatus.EMPTY: "EMPTY"
  }.get(status, None)

  if text is None:
    text = "UNKNOWN"
  socketio.emit(u"status", {
      'state': str(text),
      'queue': [str(q) for q in queue]
  },
                broadcast=True)


def broadcast_robot(vertex, seq, tf_leaf_trunk, cov_leaf_trunk, target_vertex,
                    tf_leaf_target, cov_leaf_target):
  """Broadcasts socketIO messages to all clients on position change of the robot

  :param vertex Current closest vertex ID to the robot
  :param seq The integer sequence of the robot along the current localization chain
  :param tf_leaf_trunk Transform from robot to trunk
  :param cov_leaf_trunk Covariance (diagonal) of the robot to trunk transform
  :param target_vertex The target vertex we are trying to merge into
  :param tf_leaf_target Transform from robot to target
  :param cov_leaf_target Covariance of the robot to target transform
  """
  log.info('Broadcast robot')
  status = graph_pb2.RobotStatus()
  status.vertex = vertex
  status.seq = seq
  (status.tf_leaf_trunk.x, status.tf_leaf_trunk.y,
   status.tf_leaf_trunk.theta) = tf_leaf_trunk

  for val in cov_leaf_trunk:
    status.cov_leaf_trunk.append(val)

  if 0 <= target_vertex < 2**64 - 1:
    status.target_vertex = target_vertex
    (status.tf_leaf_target.x, status.tf_leaf_target.y,
     status.tf_leaf_target.theta) = tf_leaf_target

    for val in cov_leaf_target:
      status.cov_leaf_target.append(val)

  socketio.emit(u"robot/loc", bytes(status.SerializeToString()), broadcast=True)


def broadcast_path(path):
  """Broadcasts socketIO messages to all clients on position change of the robot

  :param path List of vertices representing the current localization chain
  """
  log.info("Broadcasting new path %s", str(path))
  socketio.emit(u"robot/path", {'path': path}, broadcast=True)


def broadcast_graph(msg):
  """Broadcasts socketIO messages to all clients on updates to the graph

  :param msg Update message from ROS
  """
  log.info('Broadcast graph')
  update = graph_pb2.GraphUpdate()
  update.seq = msg['seq']
  update.stamp = msg['stamp']
  update.invalidate = msg['invalidate']

  for v in msg['vertices']:
    vertex = update.vertices.add()
    vertex.id = v['id']
    vertex.lat = v['T_projected'][1]
    vertex.lng = v['T_projected'][0]
    vertex.theta = v['T_projected'][2]
    vertex.neighbours.extend(v['neighbours'])

  socketio.emit(u"graph/update",
                bytes(update.SerializeToString()),
                broadcast=True)


def broadcast_safety(msg):
  """Broadcasts socketIO messages to all clients on safety monitor updates

  :param msg Status message from ROS
  """
  log.info('Broadcast safety status')
  # status = SafetyStatus()
  # status.signal = msg['signal']
  # status.action = msg['action']

  # socketio.emit(u"safety", bytes(status.SerializeToString()), broadcast=True)


def broadcast_new_dialog(dialog_dict):
  log.info('Broadcast new dialog')
  # socketio.emit(u"dialog/new", dialog_dict, broadcast=True)


def broadcast_cancel_dialog(dialog_id):
  log.info('Broadcast cancel dialog')
  # socketio.emit(u"dialog/cancel", {"id": dialog_id}, broadcast=True)


def broadcast_replace_dialog(dialog_dict):
  log.info('Broadcast replace dialog')
  # socketio.emit(u"dialog/replace", dialog_dict, broadcast=True)


def broadcast_finish_dialog(dialog_id):
  log.info('Broadcast finish dialog')
  # socketio.emit(u"dialog/finish", {"id": dialog_id}, broadcast=True)


def broadcast_overlay():
  log.info('Broadcast overlay')
  # socketio.emit(u"overlay/refresh")


@app.route('/')
def main():
  return "This is a socket-only API server."


if __name__ == '__main__':
  log.info("Launching socket server.")

  # TODO: Server runs on all interfaces.  Can we assume a trusted network?
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)