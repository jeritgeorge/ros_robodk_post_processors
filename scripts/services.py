#!/usr/bin/env python
package = 'ros_robodk_post_processors'
service_base_name = "/robodk_post_processors/"

from robodk_postprocessors.Fanuc_R30iA import RobotPost as FanucR30iAPost
from robodk_postprocessors.Motoman import Pose
from robodk_postprocessors.Motoman import RobotPost as MotomanPost
from robodk_postprocessors.robodk import *
from ros_robodk_post_processors.srv import *
import config
import geometry_msgs.msg
import trajectory_msgs.msg
from rpgatta_msgs.msg import MotionPlan
from rpgatta_msgs.msg import ProcessSegment
from rpgatta_msgs.msg import TransitionPair
from rpgatta_msgs.msg import RobotProcessPath
from rpgatta_msgs.msg import ProcessType
import motoman
import rospy

previousTool = 0
previousToolSetting = -1  #-1 is off, 0-1 for minimum to maximum work

pnsNumber = "0010"  #0010

processToTool = {
		ProcessType.NONE : 0,
		ProcessType.CHEMICAL_DEPAINT_AGITATE : 1,
		ProcessType.CHEMICAL_DEPAINT_STRIP : 11,
		ProcessType.CHEMICAL_DEPAINT_WASH : 3,
		ProcessType.GRIT_BLAST_STRIP : 1,
		ProcessType.SAND_STRIP : 1
	}
AGITATORRPM_MIN = 5
AGITATORRPM_MAX = 20
HPRPM_MIN = 1000
HPRPM_MAX = 2500
HPPRESSURE_MIN = 3000
HPPRESSURE_MAX = 15000

# From geometry_msgs.Pose to RoboDK.Mat
def poseToMat(p):
    # Warning: Quaternion initialization order: w, x, y, z
    quat = [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
    mat = quaternion_2_pose(quat)
    mat.setPos([p.position.x * 1000, p.position.y * 1000, p.position.z * 1000])
    return mat

# Check if pose is initialized
def poseInitialized(p):
    if p.orientation.x == 0 and p.orientation.y == 0 and p.orientation.z == 0 and p.orientation.w == 0:
        return False
    return True

def move_c(req):
    if config.pp is None:
        return [config.pp_not_init]

    if len(req.joints_1) != config.pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.joints_2) != config.pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.conf_RLF_1) != 3:
        return ["conf_RLF_1 size must be 3"]
    if len(req.conf_RLF_2) != 3:
        return ["conf_RLF_2 size must be 3"]

    if not poseInitialized(req.pose_1):
        return ["Pose 1 quaternions are not initialized"]
    if not poseInitialized(req.pose_2):
        return ["Pose 2 quaternions are not initialized"]
    p1 = poseToMat(req.pose_1)
    p2 = poseToMat(req.pose_2)
    config.pp.MoveC(p1, req.joints_1, p2, req.joints_2, list(req.conf_RLF_1), list(req.conf_RLF_2))
    return [""]

def move_j(req):
    if config.pp is None:
        return [config.pp_not_init]

    if len(req.joints) != config.pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.conf_RLF) != 3:
        return ["conf_RLF size must be 3"]

    if not poseInitialized(req.pose):
        return ["Pose quaternions are not initialized"]
    p = poseToMat(req.pose)
    config.pp.MoveJ(p, req.joints, list(req.conf_RLF))
    return [""]

def move_l(req):
    if config.pp is None:
        return [config.pp_not_init]

    if len(req.joints) != config.pp.nAxes:
        return ["Joints tuple does not match the number of axes"]
    if len(req.conf_RLF) != 3:
        return ["conf_RLF size must be 3"]

    if not poseInitialized(req.pose):
        #return ["Pose quaternions are not initialized"]
        p = None
    else:
        p = poseToMat(req.pose)
    config.pp.MoveL(p, req.joints, list(req.conf_RLF))
    return [""]

def pause(req):
    if config.pp is None:
        return [config.pp_not_init]

    if req.seconds <= 0.0:
        return ["Pause cannot be zero or negative"];

    config.pp.Pause(req.seconds * 1000)
    return [""]

def prog_finish(req):
    if config.pp is None:
        return [config.pp_not_init]

    config.pp.ProgFinish(req.program_name)
    if len(config.pp.LOG) > 0:
        return [config.pp.LOG, ""]

    program=''
    for line in config.pp.PROG_LIST[-1]:
        program += line
    return ["", program]

def prog_save(req):
    if config.pp is None:
        return [config.pp_not_init, ""]

    if len(config.pp.PROG_LIST) is 0:
        return ["Program list is empty", ""]

    program=''
    for line in config.pp.PROG_LIST[-1]:
        program += line

    if not req.program_name:
        return ["program_name cannot be empty", ""]
    if not req.file_saving_dir:
        return ["file_saving_dir cannot be empty", ""]
    config.pp.ProgSave(req.file_saving_dir, req.program_name)
    return ["", program]

def prog_send_robot(req):
    if config.pp is None:
        return [config.pp_not_init, ""]

    if not req.robot_ip:
        return ["robot_ip cannot be empty"]

    config.pp.ProgSendRobot(req.robot_ip, req.remote_path, req.ftp_user, req.ftp_pass)
    return [""]

def prog_start(req):
    if len(req.post_processor) is 0:
        return ["Post processor name is empty"]
    elif req.post_processor == "Motoman":
        config.pp = MotomanPost()
        config.pp.LAST_CONFDATA = [None, None, None, None] # Reset last configuration
    elif req.post_processor == "Fanuc_R30iA":
        config.pp = FanucR30iAPost()
    # New post-processors go here
    else:
        return ["'%s' post processor is not supported" % req.post_processor]

    config.pp.PROG_TARGETS = []

    if len(req.program_name) is 0:
        return ["Program name is empty"]
    config.pp.ProgStart(req.program_name)

    # Program comment is allowed to be empty
    config.pp.PROG_COMMENT = req.program_comment

    return [""]

def run_message(req):
    if config.pp is None:
        return [config.pp_not_init]

    if not req.msg:
        return ["message cannot be empty"]

    config.pp.RunMessage(req.msg)
    return [""]

def run_code(req):
    if config.pp is None:
        return [config.pp_not_init]

    if not req.code:
        return ["code cannot be empty"]

    config.pp.RunCode(req.code, req.is_function_call)
    return [""]

def set_do(req): # set Digital Output
    if config.pp is None:
        return [config.pp_not_init]

    try:
        var = int(req.io_var)
        config.pp.setDO(var, req.io_value)
    except ValueError:
        config.pp.setDO(req.io_var, req.io_value)

    return [""]

def set_go(req): # set Group Output
    if config.pp is None:
        return [config.pp_not_init]

    try:
        var = int(req.io_var)
        config.pp.setGO(var, req.io_value)
    except ValueError:
        config.pp.setGO(req.io_var, req.io_value)

    return [""]

def set_frame(req):
    if config.pp is None:
        return [config.pp_not_init]

    if not poseInitialized(req.pose):
        return ["Pose quaternions are not initialized"]
    p = poseToMat(req.pose)
    config.pp.setFrame(p, req.frame_id, req.frame_name)
    return [""]

def set_speed(req):
    if config.pp is None:
        return [config.pp_not_init]

    config.pp.setSpeed(req.mm_sec)
    return [""]

def set_speed_joints(req):
    if config.pp is None:
        return [config.pp_not_init]

    config.pp.setSpeedJoints(req.deg_sec)
    return [""]

def set_tool(req):
    if config.pp is None:
        return [config.pp_not_init]

    if not poseInitialized(req.pose):
        return ["Pose quaternions are not initialized"]
    p = poseToMat(req.pose)
    config.pp.setTool(p, req.tool_id, req.tool_name)
    return [""]

def set_zone_data(req):
    if config.pp is None:
        return [config.pp_not_init]

    config.pp.setZoneData(req.zone_mm)
    return [""]

def wait_di(req):
    if config.pp is None:
        return [config.pp_not_init]

    try:
        var = int(req.io_var)
        config.pp.waitDI(var, req.io_value, req.timeout_ms)
    except ValueError:
        config.pp.waitDI(req.io_var, req.io_value, req.timeout_ms)

    return [""]

#---------------------High level LS generation services-------------------
def createProgramName(number):
    return "ROS" + pnsNumber + "_" + str(number)

def turnToolOn():
    #------set_do-----
    service = service_base_name + "set_do"
    srv = rospy.ServiceProxy(service, SetDO)
    success = False
    try:
        resp = srv('96', True)
        resp = srv('97', False)
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #------wait_di-----
    service = service_base_name + "wait_di"
    srv = rospy.ServiceProxy(service, WaitDI)
    success = False
    try:
        resp = srv('96', True, -1) #last value is timeout, enter less than 0 for no timeout
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #------set_do-----
    service = service_base_name + "set_do"
    srv = rospy.ServiceProxy(service, SetDO)
    success = False
    try:
        resp = srv('96', False)
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

def turnToolOff():
    #------set_do-----
    service = service_base_name + "set_do"
    srv = rospy.ServiceProxy(service, SetDO)
    success = False
    try:
        resp = srv('97', True)
        resp = srv('96', False)
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #------wait_di-----
    service = service_base_name + "wait_di"
    srv = rospy.ServiceProxy(service, WaitDI)
    success = False
    try:
        resp = srv('97', True, -1) #last value is timeout, enter less than 0 for no timeout
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #------set_do-----
    service = service_base_name + "set_do"
    srv = rospy.ServiceProxy(service, SetDO)
    success = False
    try:
        resp = srv('97', False)
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

def setTool(Tool):
    #------set_go-----
        service = service_base_name + "set_go"
        srv = rospy.ServiceProxy(service, SetGO)
        success = False
        try:
            resp = srv('12', str(Tool)) #GO[12] is tool selection
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

def setAgitatorRPM(work):
    #------set_go-----
    rpm = max(min(((AGITATORRPM_MAX-AGITATORRPM_MIN) * work) + AGITATORRPM_MIN, AGITATORRPM_MAX),AGITATORRPM_MIN)
    service = service_base_name + "set_go"
    srv = rospy.ServiceProxy(service, SetGO)
    success = False
    try:
        resp = srv('5', str(rpm)) #GO[12] is tool selection
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))            

def setHighPressureWaterRPM(work):
    #------set_go-----
    rpm = ((HPRPM_MAX-HPRPM_MIN) * work) + HPRPM_MIN 
    service = service_base_name + "set_go"
    srv = rospy.ServiceProxy(service, SetGO)
    success = False
    try:
        resp = srv('6', str(rpm)) #GO[12] is tool selection
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))  

def setHighPressureWaterPressure(work):
    #------set_go-----
    p = ((HPPRESSURE_MAX-HPPRESSURE_MIN) * work) + HPPRESSURE_MIN 
    service = service_base_name + "set_go"
    srv = rospy.ServiceProxy(service, SetGO)
    success = False
    try:
        resp = srv('7', str(p)) #GO[12] is tool selection
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))  

def generate_robot_program(req):
    global previousTool
    global previousToolSetting
    previousTool = 0
    previousToolSetting = -1
    data = req.plan
    programNameList = []
    try:
        if data.from_home is not None:
            programNameList.append([createProgramName(len(programNameList)+1), "fromHome"])
            createLSfromRobotProcessPath(data.from_home, programNameList[-1][0], programNameList[-1][1])
    except rospy.ServiceException as exc:
        rospy.logerr("error: " + str(exc))

    try:
        i = 0
        for segment in data.segments:
            
            if segment is not None and not (segment == ProcessSegment()):
                rospy.loginfo("%s" % segment)
                if segment.approach is not None and not (segment.approach == RobotProcessPath()):
                    programNameList.append([createProgramName(len(programNameList)+1),"approach" + str(i)])
                    createLSfromRobotProcessPath(segment.approach, programNameList[-1][0], programNameList[-1][1])
                if segment.process is not None and not (segment.process == RobotProcessPath()):
                    programNameList.append([createProgramName(len(programNameList)+1),"process" + str(i)])
                    createLSfromRobotProcessPath(segment.process, programNameList[-1][0], programNameList[-1][1])
                if segment.departure is not None and not (segment.departure == RobotProcessPath()):
                    programNameList.append([createProgramName(len(programNameList)+1),"departure" + str(i)])
                    createLSfromRobotProcessPath(segment.departure, programNameList[-1][0], programNameList[-1][1])
            i = i + 1
    except rospy.ServiceException as exc:
        rospy.logerr("error: " + str(exc))     

    try:
        if data.to_home is not None:
            programNameList.append([createProgramName(len(programNameList)+1),"toHome"])
            createLSfromRobotProcessPath(data.to_home, programNameList[-1][0], programNameList[-1][1])
    except rospy.ServiceException as exc:
        rospy.logerr("error: " + str(exc))   

    createMasterLS("pns" + pnsNumber, programNameList)

def createMasterLS(prgname, programList):
    #-----prog_start-----
    service = service_base_name + "prog_start"
    srv = rospy.ServiceProxy(service, ProgStart)
    success = False
    try:
        resp = srv("Fanuc_R30iA", prgname, "Master ROS Program")
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #for each point in the trajectory
    for indx, programName in enumerate(programList):
        #------run_code-----
        service = service_base_name + "run_code"
        srv = rospy.ServiceProxy(service, RunCode)
        success = False
        try:
            resp = srv(programName[0], True)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

        

    #------prog_finish-----
    service = service_base_name + "prog_finish"
    srv = rospy.ServiceProxy(service, ProgFinish)
    success = False
    try:
        resp = srv(prgname)
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))
    

    #------prog_save-----
    service = service_base_name + "prog_save"
    srv = rospy.ServiceProxy(service, ProgSave)
    success = False
    try:
        resp = srv(prgname, "/home/controls/catkin_ws/src/ros_robodk_post_processors")
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

def createLSfromRobotProcessPath(data, prgname, prgcomment): 
#---------------------End High level LS generation services --------------

# Common services
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
    services.append(rospy.Service(service_prefix + 'set_go', SetGO, set_go))
    services.append(rospy.Service(service_prefix + 'set_frame', SetFrame, set_frame))
    services.append(rospy.Service(service_prefix + 'set_speed', SetSpeed, set_speed))
    services.append(rospy.Service(service_prefix + 'set_speed_joints', SetSpeedJoints, set_speed_joints))
    services.append(rospy.Service(service_prefix + 'set_tool', SetTool, set_tool))
    services.append(rospy.Service(service_prefix + 'set_zone_data', SetZoneData, set_zone_data))
    services.append(rospy.Service(service_prefix + 'wait_di', WaitDI, wait_di))
    services.append(rospy.Service(service_prefix + 'generate_robot_program', GenerateRobotProgram, generate_robot_program ))

    # Not implemented:
    # setSpeed
    # setAcceleration
    # setSpeedJoints
    # setAccelerationJoints
    # setZoneData

    # Brand specific services
    motoman.services(service_prefix, services)

    # Display all services advertised
    for s in services:
        rospy.loginfo("Service %s ready", s.resolved_name)
    rospy.spin()

if __name__ == '__main__':
    services_servers()
