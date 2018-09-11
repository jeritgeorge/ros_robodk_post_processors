#!/usr/bin/env python
package = 'ros_robodk_post_processors'
name = 'motoman'
service_base_name = "/robodk_post_processors"

from ros_robodk_post_processors.srv import *
import rospy
import unittest

class TestMotoman(unittest.TestCase):

    def testWaitForServices(self):
        service_available = False
        # /robodk_post_processors/move_c

        service_name = service_base_name + "/move_c"
        try:
            service_available  = rospy.wait_for_service(service_name, 3)
        except:
            print("Could not connect to service %s" % service_name)

        print(service_available)

        self.assertEquals(service_available, True, "Service %s is not available!" % service_name)

        #service_name = "/move_j"
        #service_available  = rospy.wait_for_service(service_name, 10)
        #print(service_available)
        #self.assertEquals(service_available, True, "Service %s is not available" % service_name)

        #service_name = "/move_l"
        #service_available  = rospy.wait_for_service(service_name, 10)
        #print(service_available)
        #self.assertEquals(service_available, True, "Service %s is not available" % service_name)

        #except rospy.ServiceException as e:
            #print("success -- ros exception was thrown: %s" % e)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, name, TestMotoman, sys.argv)
