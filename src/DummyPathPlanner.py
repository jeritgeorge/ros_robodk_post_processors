#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rpgatta_msgs.msg import MotionPlan
from rpgatta_msgs.msg import ProcessSegment
from rpgatta_msgs.msg import TransitionPair
from rpgatta_msgs.msg import RobotProcessPath
from rpgatta_msgs.msg import ProcessType
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', MotionPlan, queue_size=10)
    rospy.init_node('talker', anonymous=True)
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
        processType.val = processType.CHEMICAL_DEPAINT_AGITATE

        processTypeMove = ProcessType()
        processTypeMove.val = processType.NONE

        
        trajectory = JointTrajectory()

        point=JointTrajectoryPoint()
        point2=JointTrajectoryPoint()

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

        point.positions.append(0.0)
        point.positions.append(0.0)
        point.positions.append(0.3)
        point.positions.append(0.6)
        point.positions.append(0.5)
        point.positions.append(10.0)
        point.positions.append(1000.0)
        point.positions.append(1000.0)
        point.velocities.append(20.0)

        point2.positions.append(0.0)
        point2.positions.append(0.0)
        point2.positions.append(0.0)
        point2.positions.append(0.0)
        point2.positions.append(0.0)
        point2.positions.append(0.0)
        point2.positions.append(1000.0)
        point2.positions.append(1000.0)
        point2.velocities.append(15.0)

        trajectory.points.append(point)
        trajectory.points.append(point2)

        examplePath.trajectory = trajectory
        examplePath.velocity = [100, 101.1]
        examplePath.tool_work = [1, 1]
        examplePath.type = processTypeMove

        examplePathProcess.trajectory = trajectory
        examplePathProcess.velocity = [100, 101.1]
        examplePathProcess.tool_work = [1, 1]
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
        #initialize transitions
        exampleTransition = TransitionPair()
        exampleTransition.front = examplePath
        exampleTransition.back = examplePath
        transitions.append(exampleTransition)

        #asign parts to motionPLan
        motionPlan.from_home = fromHome
        motionPlan.segments = segments
        motionPlan.transitions = transitions
        motionPlan.to_home = toHome

        pub.publish(motionPlan)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
