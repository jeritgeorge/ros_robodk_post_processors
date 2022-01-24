#!/usr/bin/env python
from ros_robodk_post_processors.robodk_post_processors.Motoman import Pose
from ros_robodk_post_processors.robodk_post_processors.robodk import *
from ros_robodk_post_processors.srv import *
from ros_robodk_post_processors import config
import geometry_msgs.msg
import rospy

not_motoman = "Cannot use on a non Motoman post processor"

def services(service_prefix, services):
    motoman_prefix = 'motoman/'
    services.append(rospy.Service(service_prefix + motoman_prefix + 'arcof', Arcof, arcof))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'arcon', Arcon, arcon))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'macro', Macro, macro))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'dont_use_mframe', DontUseMFrame, dontUseMFrame))
    services.append(rospy.Service(service_prefix + motoman_prefix + 'set_folder', SetFolder, setFolder))

def isMotomanPP():
    if config.pp is None:
      return False

    if config.pp.PROG_EXT != "JBI":
      return False

    return True

def arcof(req):
    if config.pp is None:
        return [config.pp_not_init]
    if not isMotomanPP():
        return [not_motoman]

    config.pp.Arcof(req.aef_file)
    return [""]

def arcon(req):
    if config.pp is None:
        return [config.pp_not_init]
    if not isMotomanPP():
        return [not_motoman]

    config.pp.Arcon(req.asf_file)
    return [""]

def macro(req):
    if config.pp is None:
        return [config.pp_not_init]
    if not isMotomanPP():
        return [not_motoman]

    if req.number is 0:
        return ["number cannot be zero"]

    if req.mf is 0:
        return ["mdf cannot be zero"]

    config.pp.Macro(req.number, req.mf, req.args)
    return [""]

def dontUseMFrame(req):
    if config.pp is None:
        return [config.pp_not_init]
    if not isMotomanPP():
        return [not_motoman]

    config.pp.DONT_USE_MFRAME = req.value;
    return [""]

def setFolder(req):
    if config.pp is None:
        return [config.pp_not_init]
    if not isMotomanPP():
        return [not_motoman]

    config.pp.SetFolder(req.folder)
    return [""]
