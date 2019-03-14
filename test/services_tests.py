#!/usr/bin/env python
package = 'ros_robodk_post_processors'
service_base_name = "/robodk_post_processors/"

from ros_robodk_post_processors.srv import *
import geometry_msgs.msg
import rospy
import unittest

def checkService(service_name):
    service_available = False
    try:
        rospy.wait_for_service(service_name, 1)
        service_available = True
    except:
        rospy.logerr("Could not connect to service %s" % service_name)
    return service_available

class ServicesTests(unittest.TestCase):

    def testWaitForServices(self):
        services = ["move_c", "move_j", "move_l", "pause", "prog_finish", "prog_save", "prog_send_robot", "prog_start", "run_code", "run_message", "set_do", "set_go", "set_frame", "set_speed", "set_speed_joints", "set_tool", "set_zone_data", "wait_di"]

        for name in services:
            service = service_base_name + name
            self.assertEquals(checkService(service), True, "Service %s is not available!" % service)

    def testFanucProgram(self):
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

        #------prog_start-----
        service = service_base_name + "prog_start"
        srv = rospy.ServiceProxy(service, ProgStart)
        success = False
        try:
            resp = srv("Fanuc_R30iA", "test", "")
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------set_tool-----
        service = service_base_name + "set_tool"
        srv = rospy.ServiceProxy(service, SetTool)
        success = False
        try:
            resp = srv(0, "tool", geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)))
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------set_frame-----
        service = service_base_name + "set_frame"
        srv = rospy.ServiceProxy(service, SetFrame)
        success = False
        try:
            resp = srv(1, "frame", geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)))
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))


        #------move_c-----
        # service = service_base_name + "move_c"
        # srv = rospy.ServiceProxy(service, MoveC)
        # success = False
        # try:
        #     resp = srv(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(1, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
        #                [0, 0, 0, 0, 0, 0],
        #                [0, 0, 0],
        #                geometry_msgs.msg.Pose(geometry_msgs.msg.Point(1.5, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
        #                [0, 0, 0, 0, 0, 0],
        #                [0, 0, 0])
        #     success = True
        # except rospy.ServiceException as exc:
        #     rospy.logerr("Service did not process request: " + str(exc))
        # self.assertEquals(success, True, "Failed to call service %s" % srv)
        # self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------set_speed_joints-----
        service = service_base_name + "set_speed_joints"
        srv = rospy.ServiceProxy(service, SetSpeedJoints)
        success = False
        try:
            resp = srv(20.0) #takes in degrees/sec, inserts % speed for joint moves
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------move_j-----
        service = service_base_name + "move_j"
        srv = rospy.ServiceProxy(service, MoveJ)
        success = False
        try:
            resp = srv(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(1, 0, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
                       [0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0])
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------set_zone_data-----
        service = service_base_name + "set_zone_data"
        srv = rospy.ServiceProxy(service, SetZoneData)
        success = False
        try:
            resp = srv(2.0)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------set_speed-----
        service = service_base_name + "set_speed"
        srv = rospy.ServiceProxy(service, SetSpeed)
        success = False
        try:
            resp = srv(20.0)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------move_l-----
        service = service_base_name + "move_l"
        srv = rospy.ServiceProxy(service, MoveL)
        success = False
        try:
            resp = srv(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(1, 0.5, 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
                       [0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0])
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------move_l-----joint position
        service = service_base_name + "move_l"
        srv = rospy.ServiceProxy(service, MoveL)
        success = False
        try:
            resp = srv(None,
                       [0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0])
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------run_message-----
        service = service_base_name + "run_message"
        srv = rospy.ServiceProxy(service, RunMessage)
        success = False
        try:
            resp = srv("A run message")
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------pause-----
        service = service_base_name + "pause"
        srv = rospy.ServiceProxy(service, Pause)
        success = False
        try:
            resp = srv(1.0)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------set_do-----
        service = service_base_name + "set_do"
        srv = rospy.ServiceProxy(service, SetDO)
        success = False
        try:
            resp = srv('1', True)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------set_go-----
        service = service_base_name + "set_go"
        srv = rospy.ServiceProxy(service, SetGO)
        success = False
        try:
            resp = srv('12', '1')
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------wait_di-----
        service = service_base_name + "wait_di"
        srv = rospy.ServiceProxy(service, WaitDI)
        success = False
        try:
            resp = srv('2', True, 0)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------run_code-----
        service = service_base_name + "run_code"
        srv = rospy.ServiceProxy(service, RunCode)
        success = False
        try:
            resp = srv("MY_FUNC", False)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------prog_finish-----
        service = service_base_name + "prog_finish"
        srv = rospy.ServiceProxy(service, ProgFinish)
        success = False
        try:
            resp = srv("test")
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

        #------prog_save-----
        service = service_base_name + "prog_save"
        srv = rospy.ServiceProxy(service, ProgSave)
        success = False
        try:
            resp = srv("test", "/home/controls/catkin_ws/src/ros_robodk_post_processors")
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        self.assertEquals(success, True, "Failed to call service %s" % srv)
        self.assertEquals(len(resp.error), 0, "Service %s failed with an error: %s" % (srv, resp.error))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, "services_tests", ServicesTests, sys.argv)
