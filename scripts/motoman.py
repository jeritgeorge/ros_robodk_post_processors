#!/usr/bin/env python
from robodk_postprocessors.Motoman import Pose
from robodk_postprocessors.robodk import *
from ros_robodk_post_processors.srv import *
import config
import geometry_msgs.msg
import rospy

def services(service_prefix, services):
    motoman_prefix = 'motoman/'
    services.append(rospy.Service(service_prefix + motoman_prefix + 'arcof', Arcof, arcof))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'arcon', Arcon, arcon))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'macro', Macro, macro))

def arcof(req):
    if config.pp is None:
        return [config.pp_not_init]

    config.pp.Arcof(req.aef_file)
    return [""]

def arcon(req):
    if config.pp is None:
        return [config.pp_not_init]

    config.pp.Arcon(req.asf_file)
    return [""]

def macro(req):
    if config.pp is None:
        return [config.pp_not_init]

    if req.number is 0:
        return ["number cannot be zero"]

    if req.mf is 0:
        return ["mdf cannot be zero"]

    config.pp.Macro(req.number, req.mf, req.args)
    return [""]
