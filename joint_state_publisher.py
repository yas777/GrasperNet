#!/usr/bin/env python
import rospy
import sys
import os
from sensor_msgs.msg import JointState

# sys.path.append(os.path.abspath("../"))
# print(sys.path)
from robot import HelloRobot


def publish_joint_states():
    rospy.init_node('joint_state_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    robot = HelloRobot()

    rate = rospy.Rate(10)  # Publish joint states at 10 Hz (adjust as needed)

    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        # joint_state.name = ["joint1", "joint2", "joint3"]  # Replace with your joint names
        # joint_state.position = [0.0, 0.0, 0.0]  # Replace with your joint positions

        names, positions = robot.get_joints()
        joint_state.name = names
        joint_state.position = positions

        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
