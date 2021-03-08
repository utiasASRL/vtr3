#!/usr/bin/env python2

import rospy

from asrl__messages.msg import DesiredActionIn

class PlaybackSafetyMonitor(object):

  pub_status = None
  status_timer = None

  def __init__(self):

    self.pub_status = rospy.Publisher('/safety_monitor_node/out/desired_action',
                                      DesiredActionIn,
                                      queue_size=1)
    self.status_timer = rospy.Timer(period=rospy.Duration(0.2),
                                    callback=self.sendStatus,
                                    oneshot=False)

  def sendStatus(self, event):
    msg = DesiredActionIn()

    msg.desired_action = "CONTINUE"
    msg.speed_limit = 4.5

    self.pub_status.publish(msg)


if __name__ == "__main__":

  rospy.init_node('safety_monitor_node')
  controller_node = PlaybackSafetyMonitor()

  rospy.spin()