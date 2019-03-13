#!/usr/bin/env python
package = 'ros_robodk_post_processors'
service_base_name = "/robodk_post_processors/"

from ros_robodk_post_processors.srv import *
import geometry_msgs.msg
import trajectory_msgs.msg
import rospy
import unittest 

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)     
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", trajectory_msgs.msg.JointTrajectory, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()