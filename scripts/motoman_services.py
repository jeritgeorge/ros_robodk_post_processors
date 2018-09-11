#!/usr/bin/env python
from robodk_postprocessors.Fanuc_R30iA import RobotPost as FanucR30iAPost
from robodk_postprocessors.Motoman import Pose
from robodk_postprocessors.Motoman import RobotPost as MotomanPost
from robodk_postprocessors.robodk import *
from ros_robodk_post_processors.srv import *
import geometry_msgs.msg
import rospy

def motomanServices(service_prefix, services):
    motoman_prefix = 'motoman/'
    services.append(rospy.Service(service_prefix + motoman_prefix + 'arcof', Arcof, arcof))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'arcon', Arcon, arcon))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'macro', Macro, macro))

# FIXME
def arcof():
    pass

# FIXME
def arcon():
    pass

# FIXME
def macro():
    pass
