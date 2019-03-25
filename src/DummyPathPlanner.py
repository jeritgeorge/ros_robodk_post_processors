#!/usr/bin/env python
# license removed for brevity
import rospy
from ros_robodk_post_processors.srv import *
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rpgatta_msgs.msg import MotionPlan
from rpgatta_msgs.msg import ProcessSegment
from rpgatta_msgs.msg import TransitionPair
from rpgatta_msgs.msg import RobotProcessPath
from rpgatta_msgs.msg import ProcessType
from std_msgs.msg import String

service_base_name = "/robodk_post_processors/"

def talker():
    pub = rospy.Publisher('chatter', MotionPlan, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.wait_for_service(service_base_name + "generate_robot_program")
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        motionPlan = MotionPlan()
        fromHome = RobotProcessPath()
        segments = []
        transitions = []
        toHome = RobotProcessPath()
        examplePath = RobotProcessPath()
        examplePathProcess = RobotProcessPath()
        processType = ProcessType()
        processType.val = ProcessType.CHEMICAL_DEPAINT_WASH

        processTypeMove = ProcessType()
        processTypeMove.val = ProcessType.NONE

        
        trajectory = JointTrajectory()

        point=JointTrajectoryPoint()
        point2=JointTrajectoryPoint()
        point3=JointTrajectoryPoint()

        trajectory.header.stamp=rospy.Time.now()
        trajectory.header.frame_id = "/base_link"

        trajectory.joint_names.append("joint_1")
        trajectory.joint_names.append("joint_2")
        trajectory.joint_names.append("joint_3")
        trajectory.joint_names.append("joint_4")
        trajectory.joint_names.append("joint_5")
        trajectory.joint_names.append("joint_6")
        trajectory.joint_names.append("joint_7")
        trajectory.joint_names.append("joint_8")

        point.positions.append(-90.0)
        point.positions.append(-65.0)
        point.positions.append(0.0)
        point.positions.append(-180.0)
        point.positions.append(-60.0)
        point.positions.append(10.0)
        point.positions.append(-1000.0)
        point.positions.append(-1000.0)
        point.velocities.append(20.0)

        point2.positions.append(-90.0)
        point2.positions.append(-65.0)
        point2.positions.append(0.0)
        point2.positions.append(-180.0)
        point2.positions.append(-60.0)
        point2.positions.append(0.0)
        point2.positions.append(-1500.0)
        point2.positions.append(-1500.0)
        point2.velocities.append(15.0)

        point3.positions.append(-90.0)
        point3.positions.append(-65.0)
        point3.positions.append(0.0)
        point3.positions.append(-180.0)
        point3.positions.append(-60.0)
        point3.positions.append(0.0)
        point3.positions.append(-2000.0)
        point3.positions.append(-2000.0)
        point3.velocities.append(15.0)

        trajectory.points.append(point)
        trajectory.points.append(point2)
        trajectory.points.append(point3)

        examplePath.trajectory = trajectory
        examplePath.velocity = [100, 200, 100]
        examplePath.tool_work = [-1, -1, -1]
        examplePath.type = processTypeMove

        examplePathProcess.trajectory = trajectory
        examplePathProcess.velocity = [100, 200, 100]
        examplePathProcess.tool_work = [-1, 0, 1]
        examplePathProcess.type = processType

        #initialize fromHome
        fromHome = examplePath
        #initialize toHome
        toHome = examplePath
        #initialize semgnets
        exampleSegment = ProcessSegment()
        exampleSegment.approach = examplePathProcess
        exampleSegment.process = examplePathProcess
        exampleSegment.departure = examplePathProcess
        segments.append(exampleSegment)

        exampleSegment = ProcessSegment()
        #exampleSegment.approach = examplePathProcess
        exampleSegment.process = examplePathProcess
        #exampleSegment.departure = examplePathProcess
        segments.append(exampleSegment)
        #initialize transitions
        exampleTransition = TransitionPair()
        exampleTransition.from_start = examplePath
        exampleTransition.from_end = examplePath
        transitions.append(exampleTransition)

        #asign parts to motionPLan
        motionPlan.from_home = fromHome
        motionPlan.segments = segments
        motionPlan.transitions = transitions
        motionPlan.to_home = toHome

        #pub.publish(motionPlan)

        #------generate_robot_program-----
        service = service_base_name + "generate_robot_program"
        srv = rospy.ServiceProxy(service, GenerateRobotProgram)
        success = False
        try:
            resp = srv(motionPlan)
            success = True
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
