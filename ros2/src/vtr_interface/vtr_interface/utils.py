# import logging

# import rospy

# from asrl__pose_graph.srv import RelaxationService, CalibrationService

# log = logging.getLogger()
# log.setLevel(logging.INFO)


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


# def get_graph(seq):
#   """Get the relaxed pose graph from the map server"""
#   log.info("Fetching graph...")
#   return generic_request('/relaxed_graph', RelaxationService, int(seq), False,
#                          True)
