#!/usr/bin/env python
package = 'ros_robodk_post_processors'
service_base_name = "/robodk_post_processors/"

from ros_robodk_post_processors.srv import *
import geometry_msgs.msg
import trajectory_msgs.msg
from rpgatta_msgs.msg import MotionPlan
from rpgatta_msgs.msg import ProcessSegment
from rpgatta_msgs.msg import TransitionPair
from rpgatta_msgs.msg import RobotProcessPath
from rpgatta_msgs.msg import ProcessType
import rospy
import unittest 

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

def callback(data):
    global previousTool
    global previousToolSetting
    previousTool = 0
    previousToolSetting = -1
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
    global previousTool
    global previousToolSetting
    #-----prog_start-----
    service = service_base_name + "prog_start"
    srv = rospy.ServiceProxy(service, ProgStart)
    success = False
    try:
        resp = srv("Fanuc_R30iA", prgname, prgcomment)
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #------set_tool-----
    service = service_base_name + "set_tool"
    srv = rospy.ServiceProxy(service, SetTool)
    success = False
    try:
        resp = srv(1, "tool", geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)))
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #------set_frame-----
    service = service_base_name + "set_frame"
    srv = rospy.ServiceProxy(service, SetFrame)
    success = False
    try:
        resp = srv(0, "frame", geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)))
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))


    
    if data.type.val == ProcessType.NONE:
        if previousTool is not processToTool[ProcessType.NONE]:
            setTool(processToTool[ProcessType.NONE])
            turnToolOff()
            previousTool = processToTool[ProcessType.NONE]
        #for each point in the trajectory
        for indx, point in enumerate(data.trajectory.points):    
            #------set_speed_joints-----
            service = service_base_name + "set_speed_joints"
            srv = rospy.ServiceProxy(service, SetSpeedJoints)
            success = False
            try:
                resp = srv(point.velocities[0]) #takes in degrees/sec, inserts % speed for joint moves
                success = True
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))
                

            #------move_j-----
            service = service_base_name + "move_j"
            srv = rospy.ServiceProxy(service, MoveJ)
            success = False
            try:
                resp = srv(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(1, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
                            [point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4], point.positions[5], point.positions[6], point.positions[7]],
                            [0, 0, 0])
                success = True
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))
    else:
        if previousTool is not processToTool[data.type.val]:
            setTool(processToTool[data.type.val])
            previousTool = processToTool[data.type.val]
        
        #for each point in the trajectory
        for indx, point in enumerate(data.trajectory.points):
            #------set_tool rpm/pressure
            if not previousToolSetting == data.tool_work[indx]:
                rospy.loginfo(prgcomment + " %s" % str(indx))
                rospy.loginfo("previous setting: %s" % str(previousToolSetting))
                rospy.loginfo("new setting: %s" % str(data.tool_work[indx]))
                #if agitator set brush rpm
                if previousTool == processToTool[ProcessType.CHEMICAL_DEPAINT_AGITATE]:
                    setAgitatorRPM(data.tool_work[indx])
                #if high pressure water set rpm/pressure TODO
                
                if previousToolSetting == -1: #tool was off, so must turn on
                    turnToolOn()
                elif data.tool_work[indx] == -1: #tool was on, so must turn off
                    turnToolOff()
                previousToolSetting = data.tool_work[indx]

            #------set_speed-----
            service = service_base_name + "set_speed"
            srv = rospy.ServiceProxy(service, SetSpeed)
            success = False
            try:
                resp = srv(data.velocity[indx])
                success = True
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))

            #------move_l-----
            service = service_base_name + "move_l"
            srv = rospy.ServiceProxy(service, MoveL)
            success = False
            try:
                resp = srv(None,
                            [point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4], point.positions[5], point.positions[6], point.positions[7]],
                            [0, 0, 0])
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
        
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.wait_for_service(service_base_name + "prog_start")
    rospy.wait_for_service(service_base_name + "set_tool")
    rospy.wait_for_service(service_base_name + "set_frame")
    rospy.wait_for_service(service_base_name + "move_c")
    rospy.wait_for_service(service_base_name + "set_speed_joints")
    rospy.wait_for_service(service_base_name + "move_j")
    rospy.wait_for_service(service_base_name + "set_zone_data")
    rospy.wait_for_service(service_base_name + "set_speed")
    rospy.wait_for_service(service_base_name + "move_l")
    rospy.wait_for_service(service_base_name + "run_message")
    rospy.wait_for_service(service_base_name + "pause")
    rospy.wait_for_service(service_base_name + "set_do")
    rospy.wait_for_service(service_base_name + "set_go")
    rospy.wait_for_service(service_base_name + "run_message")
    rospy.wait_for_service(service_base_name + "wait_di")
    rospy.wait_for_service(service_base_name + "run_code")
    rospy.wait_for_service(service_base_name + "prog_finish")
    rospy.wait_for_service(service_base_name + "prog_save")
    rospy.Subscriber("chatter", MotionPlan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()