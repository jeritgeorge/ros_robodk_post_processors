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


def turnToolOn(Tool):
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

def turnToolOff(Tool):
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

def callback(data):
    try:
        if data.from_home is not None:
            createLSfromRobotProcessPath(data.from_home, "fromHome")
    except rospy.ServiceException as exc:
        rospy.logerr("error: " + str(exc))

    try:
        i = 0
        for segment in data.segments:
            createLSfromRobotProcessPath(segment.approach, "approach" + str(i))
            createLSfromRobotProcessPath(segment.process, "process" + str(i))
            createLSfromRobotProcessPath(segment.departure, "departure" + str(i))
            i = i + 1
    except rospy.ServiceException as exc:
        rospy.logerr("error: " + str(exc))     

    try:
        if data.to_home is not None:
            createLSfromRobotProcessPath(data.to_home, "toHome")
    except rospy.ServiceException as exc:
        rospy.logerr("error: " + str(exc))   

def createLSfromRobotProcessPath(data, prgname): 

    #-----prog_start-----
    service = service_base_name + "prog_start"
    srv = rospy.ServiceProxy(service, ProgStart)
    success = False
    try:
        resp = srv("Fanuc_R30iA", prgname, "test_comment")
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #-----set_go-----
    service = service_base_name + "set_go"
    srv = rospy.ServiceProxy(service, SetGO)
    success = False
    try:
        resp = srv('12', '1')
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    #------set_tool-----
    service = service_base_name + "set_tool"
    srv = rospy.ServiceProxy(service, SetTool)
    success = False
    try:
        resp = srv(0, "tool", geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)))
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    
    #------set_frame-----
    service = service_base_name + "set_frame"
    srv = rospy.ServiceProxy(service, SetFrame)
    success = False
    try:
        resp = srv(1, "frame", geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)))
        success = True
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    turnToolOn(1)
    
    #for each point in the trajectory
    for point in data.trajectory.points:
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
        
    turnToolOff(1)

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