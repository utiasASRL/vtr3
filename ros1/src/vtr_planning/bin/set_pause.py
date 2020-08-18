#!/usr/bin/env python

import rospy

from vtr_planning.srv import MissionPause

if __name__ == "__main__":
  service_name = "ShamNav/pause"
  rospy.wait_for_service(service_name)
  pause_service = rospy.ServiceProxy(service_name, MissionPause)
  pause_service(False)