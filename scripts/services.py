#!/usr/bin/env python
from robodk_postprocessors.robodk import *
from robodk_postprocessors.Motoman import Pose
from robodk_postprocessors.Motoman import RobotPost as MotomanPost
from robodk_postprocessors.Fanuc_R30iA import RobotPost as FanucR30iAPost
from ros_robodk_post_processors.srv import *
import rospy

# FIXME How to add custom commands for each post processor?
# FIXME How to handle external axes?
# FIXME How to add an option on a pose?
# FIXME How to handle multi-program generation?
# FIXME How to support modifying every option of each post processor? (options differ between post processors)

# TODO Test adding many commands to hit the maximum number of commands per program
# TODO Test performance (is using servers a good idea?)

# The post processor, global variable used in every service server
pp = None

# Error message is service is called without initializing a post-processor
pp_not_init = "No post processor initialized.\nCall prog_start service first."

def generate_native_code(req):
    global pp
    if pp is None:
        return [pp_not_init, ""]

    program=''
    for line in pp.PROG_LIST[-1]:
        program += line

    if req.save_file:
      pp.ProgSave(progname=req.program_name, folder=req.file_saving_dir)
    return ["", program]

def move_j(req):
    global pp
    if pp is None:
        return [pp_not_init]

    # FIXME Convert and use in MoveJ
    print(req.pose)
    #quaternion_2_pose
    #pose = pose_2_xyzrpw(req.pose)
    #for i in pose:
    #  print(i)

    if req.joints != pp.nAxes:
      return ["Joints tuple does not match the number of axes"]

    pp.MoveJ(Pose([200, 200, 500, 180, 0, 180]), req.joints)
    return [""]

def move_l(req):
    global pp
    if pp is None:
        return [pp_not_init]

    # FIXME Implement
    pp.MoveL(Pose([200, 200, 500, 180, 0, 180]), [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752])
    return [""]

def prog_finish(req):
    global pp
    if pp is None:
        return [pp_not_init]

    pp.ProgFinish(req.program_name)
    if len(pp.LOG) > 0:
        return [pp.LOG]
    return [""]

def prog_start(req):
    global pp
    if len(req.post_processor) is 0:
        return ["Post processor name is empty"]
    elif req.post_processor == "Motoman":
        pp = MotomanPost()
        # Default configuration for the Motoman post processor
        pp.ACTIVE_TOOL=0
        pp.USE_RELATIVE_JOB=False
    elif req.post_processor == "Fanuc_R30iA":
        pp = FanucR30iAPost()
    else:
        return ["%s post processor is not supported" % req.post_processor]

    if len(req.program_name) is 0:
        return ["Program name is empty"]
    pp.ProgStart(req.program_name)

    # Program comment can be empty
    pp.PROG_COMMENT = req.program_comment
    return [""]

def run_message(req):
    global pp
    if pp is None:
        return [pp_not_init]

    pp.RunMessage(req.msg)
    return [""]

def services_servers():
    rospy.init_node('ros_robodk_post_processors')
    service_prefix = 'robodk_post_processors/'
    services = [rospy.Service(service_prefix + 'generate_native_code', GenerateNativeCode, generate_native_code)]
    services.append(rospy.Service(service_prefix + 'move_j', MoveJ, move_j))
    services.append(rospy.Service(service_prefix + 'move_l', MoveL, move_l))
    services.append(rospy.Service(service_prefix + 'prog_finish', ProgFinish, prog_finish))
    services.append(rospy.Service(service_prefix + 'prog_start', ProgStart, prog_start))
    services.append(rospy.Service(service_prefix + 'run_message', RunMessage, run_message))
    for s in services:
        rospy.loginfo("Service %s ready", s.resolved_name)
    rospy.spin()

if __name__ == '__main__':
    services_servers()
