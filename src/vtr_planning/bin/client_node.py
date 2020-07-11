#!/usr/bin/env python

import rospy

import vtr_planning.client_proxy as client_proxy

# Do not remove this line.  Just don't.  It will cause fork bombs.
if __name__ == "__main__":
  client, mgr = client_proxy.build_master(
      server_path=rospy.remap_name('/Navigation'))

  client.start()
  mgr.get_server().serve_forever()
  client.shutdown()
