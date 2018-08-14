#!/usr/bin/env python
from motoman_services import *
from robodk_postprocessors.Fanuc_R30iA import RobotPost as FanucR30iAPost
from robodk_postprocessors.Motoman import Pose
from robodk_postprocessors.Motoman import RobotPost as MotomanPost
from robodk_postprocessors.robodk import *
from ros_robodk_post_processors.srv import *
import geometry_msgs.msg
import rospy

# The post processor, global variable used in every service server
pp = None

# Error message if a service is called prior to initializing a post-processor
pp_not_init = "No post processor initialized.\nCall prog_start service first."

# From geometry_msgs.Pose to RoboDK.Mat
def poseToMat(p):
    quat = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    mat = quaternion_2_pose(quat)
    mat.setPos([p.position.x, p.position.y, p.position.z])
    return mat

def move_c(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if len(req.joints_1) != pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.joints_2) != pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.conf_RLF_1) != 3:
        return ["conf_RLF_1 size must be 3"]
    if len(req.conf_RLF_2) != 3:
        return ["conf_RLF_2 size must be 3"]

    p1 = poseToMat(req.pose_1)
    p2 = poseToMat(req.pose_2)
    pp.MoveC(p1, req.joints_1, p2, req.joints_2, list(req.conf_RLF_1), list(req.conf_RLF_2))
    return [""]

def move_j(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if len(req.joints) != pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.conf_RLF) != 3:
        return ["conf_RLF size must be 3"]

    p = poseToMat(req.pose)
    pp.MoveJ(p, req.joints, list(req.conf_RLF))
    return [""]

def move_l(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if len(req.joints) != pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.conf_RLF) != 3:
        return ["conf_RLF size must be 3"]

    p = poseToMat(req.pose)
    pp.MoveL(p, req.joints, list(req.conf_RLF))
    return [""]

def pause(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if req.seconds <= 0.0:
        return ["Pause cannot be zero or negative"];

    pp.Pause(req.seconds * 1000)
    return [""]

def prog_finish(req):
    global pp
    if pp is None:
        return [pp_not_init]

    pp.ProgFinish(req.program_name)
    if len(pp.LOG) > 0:
        return [pp.LOG, ""]

    program=''
    for line in pp.PROG_LIST[-1]:
        program += line
    return ["", program]

def prog_save(req):
    global pp
    if pp is None:
        return [pp_not_init, ""]

    program=''
    for line in pp.PROG_LIST[-1]:
        program += line

    if req.save_file:
        if not req.program_name:
            return ["program_name cannot be empty", ""]
        if not req.file_saving_dir:
            return ["file_saving_dir cannot be empty", ""]
        pp.ProgSave(req.file_saving_dir, req.program_name)
    return ["", program]

def prog_send_robot(req):
    global pp
    if pp is None:
        return [pp_not_init, ""]

    if not req.robot_ip:
        return ["robot_ip cannot be empty"]

    pp.ProgSendRobot(req.robot_ip, req.remote_path, req.ftp_user, req.ftp_pass)
    return [""]

def prog_start(req):
    global pp
    pp = None

    if len(req.post_processor) is 0:
        return ["Post processor name is empty"]
    elif req.post_processor == "Motoman":
        pp = MotomanPost()
    elif req.post_processor == "Fanuc_R30iA":
        pp = FanucR30iAPost()
    # New post-processors go here
    else:
        return ["'%s' post processor is not supported" % req.post_processor]

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

    if not req.msg:
        return ["message cannot be empty"]

    pp.RunMessage(req.msg)
    return [""]

def run_code(req):
    global pp
    if pp is None:
        return [pp_not_init]

    if not req.code:
        return ["code cannot be empty"]

    pp.RunCode(req.code, req.is_function_call)
    return [""]

def set_do(req): # set Digital Output
    global pp
    if pp is None:
        return [pp_not_init]

    try:
        var = int(req.io_var)
        pp.setDO(var, req.io_value)
    except ValueError:
        pp.setDO(req.io_var, req.io_value)

    return [""]

def set_frame(req):
    global pp
    if pp is None:
        return [pp_not_init]

    p = poseToMat(req.pose)
    pp.setFrame(p, req.frame_id, req.frame_name)
    return [""]

def set_tool(req):
    global pp
    if pp is None:
        return [pp_not_init]

    p = poseToMat(req.pose)
    pp.setTool(p, req.tool_id, req.tool_name)
    return [""]

def wait_di(req):
    global pp
    if pp is None:
        return [pp_not_init]

    try:
        var = int(req.io_var)
        pp.waitDI(var, req.io_value, req.timeout_ms)
    except ValueError:
        pp.waitDI(req.io_var, req.io_value, req.timeout_ms)

    return [""]

def services_servers():
    rospy.init_node('ros_robodk_post_processors')
    service_prefix = 'robodk_post_processors/'
    services = []

    # Common services to all post-processors
    services.append(rospy.Service(service_prefix + 'move_c', MoveC, move_c))
    services.append(rospy.Service(service_prefix + 'move_j', MoveJ, move_j))
    services.append(rospy.Service(service_prefix + 'move_l', MoveL, move_l))
    services.append(rospy.Service(service_prefix + 'pause', Pause, pause))
    services.append(rospy.Service(service_prefix + 'prog_finish', ProgFinish, prog_finish))
    services.append(rospy.Service(service_prefix + 'prog_save', ProgSave, prog_save))
    services.append(rospy.Service(service_prefix + 'prog_send_robot', ProgSendRobot, prog_send_robot))
    services.append(rospy.Service(service_prefix + 'prog_start', ProgStart, prog_start))
    services.append(rospy.Service(service_prefix + 'run_code', RunCode, run_code))
    services.append(rospy.Service(service_prefix + 'run_message', RunMessage, run_message))
    services.append(rospy.Service(service_prefix + 'set_do', SetDO, set_do))
    services.append(rospy.Service(service_prefix + 'set_frame', SetFrame, set_frame))
    services.append(rospy.Service(service_prefix + 'set_tool', SetTool, set_tool))
    services.append(rospy.Service(service_prefix + 'wait_di', WaitDI, wait_di))
    # Not implemented:
    # setSpeed
    # setAcceleration
    # setSpeedJoints
    # setAccelerationJoints
    # setZoneData

    # Brand specific services
    motomanServices(service_prefix, services)

    # Display all services advertised
    for s in services:
        rospy.loginfo("Service %s ready", s.resolved_name)
    rospy.spin()

if __name__ == '__main__':
    services_servers()
