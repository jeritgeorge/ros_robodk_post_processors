#!/usr/bin/env python
from motoman_services import *
from robodk_postprocessors.Fanuc_R30iA import RobotPost as FanucR30iAPost
from robodk_postprocessors.Motoman import Pose
from robodk_postprocessors.Motoman import RobotPost as MotomanPost
from robodk_postprocessors.robodk import *
from ros_robodk_post_processors.srv import *
import geometry_msgs.msg
import rospy

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

# From geometry_msgs.Pose to RoboDK.Mat
def poseToMat(p):
    quat = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    mat = quaternion_2_pose(quat)
    mat.setPos([p.position.x, p.position.y, p.position.z])
    return mat

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

    if len(req.joints) != pp.nAxes:
      return ["Joints tuple does not match the number of axes"]

    p = poseToMat(req.pose)
    pp.MoveJ(p, req.joints, list(req.conf_RLF))
    return [""]

def move_l(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if len(req.joints) != pp.nAxes:
      return ["Joints tuple does not match the number of axes"]

    p = poseToMat(req.pose)
    pp.MoveL(p, req.joints, list(req.conf_RLF))
    return [""]

def move_c(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if len(req.joints_1) != pp.nAxes:
      return ["Joints tuple does not match the number of axes"]
    if len(req.joints_2) != pp.nAxes:
      return ["Joints tuple does not match the number of axes"]

    p1 = poseToMat(req.pose_1)
    p2 = poseToMat(req.pose_2)
    pp.MoveC(p1, req.joints_1, p2, req.joints_2, list(req.conf_RLF_1), list(req.conf_RLF_2))
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
    pp = None

    if len(req.post_processor) is 0:
        return ["Post processor name is empty"]
    elif req.post_processor == "Motoman":
        pp = MotomanPost()
        # Default configuration for the Motoman post processor
        # FIXME Remove
        #pp.ACTIVE_FRAME = 0
        #pp.ACTIVE_TOOL = 0
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

def pause(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if req.seconds <= 0.0:
      return ["Pause cannot be zero or negative"];

    pp.Pause(req.seconds * 1000)
    return [""]

def set_tool(req):
    global pp
    if pp is None:
        return [pp_not_init]

    p = poseToMat(req.pose)
    pp.setTool(p, req.tool_id, req.tool_name)
    return [""]

def set_frame(req):
    global pp
    if pp is None:
        return [pp_not_init]

    p = poseToMat(req.pose)
    pp.setFrame(p, req.frame_id, req.frame_name)
    return [""]

def services_servers():
    rospy.init_node('ros_robodk_post_processors')
    service_prefix = 'robodk_post_processors/'
    services = [rospy.Service(service_prefix + 'generate_native_code', GenerateNativeCode, generate_native_code)]
    services.append(rospy.Service(service_prefix + 'move_j', MoveJ, move_j))
    services.append(rospy.Service(service_prefix + 'move_l', MoveL, move_l))
    services.append(rospy.Service(service_prefix + 'move_c', MoveC, move_c))
    services.append(rospy.Service(service_prefix + 'prog_finish', ProgFinish, prog_finish))
    services.append(rospy.Service(service_prefix + 'prog_start', ProgStart, prog_start))
    services.append(rospy.Service(service_prefix + 'run_message', RunMessage, run_message))
    services.append(rospy.Service(service_prefix + 'pause', Pause, pause))
    services.append(rospy.Service(service_prefix + 'set_tool', SetTool, set_tool))
    services.append(rospy.Service(service_prefix + 'set_frame', SetFrame, set_frame))
    motomanServices(service_prefix, services)
    for s in services:
        rospy.loginfo("Service %s ready", s.resolved_name)
    rospy.spin()

if __name__ == '__main__':
    services_servers()
