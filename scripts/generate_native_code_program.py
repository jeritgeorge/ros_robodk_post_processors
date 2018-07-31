#!/usr/bin/env python

import actionlib
from ros_robodk_post_processors.srv import *
import rospy

if __name__ == '__main__':
  rospy.init_node('ros_robodk_post_processors')
  rospy.spin()
