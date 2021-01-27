import logging

# import rospy
import rclpy

# from asrl__pose_graph.srv import RelaxationService, CalibrationService
from vtr_messages.srv import GraphRelaxation

log = logging.getLogger()
log.setLevel(logging.INFO)


## Old code for references
# def generic_request(path, mtype, *args, **kwargs):
#   ns = rospy.remap_name('/Navigation') + path
#   rospy.wait_for_service(ns)
#   proxy = rospy.ServiceProxy(ns, mtype)

#   try:
#     resp = proxy(*args, **kwargs)
#   except Exception as e:
#     log.error("An error occurred: " + str(e))
#     return None

#   return resp
def ros_service_request(node, path, mtype, request):
  ros_service = node.create_client(mtype, path)
  while not ros_service.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')
  response = ros_service.call_async(request)
  rclpy.spin_until_future_complete(node, response)

  return response.result()

def get_graph(node, seq):
  """Get the relaxed pose graph from the map server"""
  log.info("Fetching graph with sequence number: {}".format(seq))

  request = GraphRelaxation.Request()
  request.seq = int(seq)
  request.update_graph = False
  request.project = True
  return ros_service_request(node, "relaxed_graph", GraphRelaxation, request)
  # return generic_request('/relaxed_graph', RelaxationService, int(seq), False,
  #                        True)
