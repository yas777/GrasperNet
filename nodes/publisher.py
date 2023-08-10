#!/usr/bin/env python
import rospy
import sys
import os
from sensor_msgs.msg import JointState

# sys.path.append(os.path.abspath("../"))
# print(sys.path)
from robot import HelloRobot


class JointStatePublisher():

    def __init__(self, hello_robot = None):
        
        # rospy.init_node('joint_state_publisher')
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        if hello_robot is None:
            self.hello_robot = HelloRobot()
        else:
            self.hello_robot = hello_robot

        self.rate = rospy.Rate(10)  # Publish joint states at 10 Hz (adjust as needed)

    def publish(self):

        while not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            # joint_state.name = ["joint1", "joint2", "joint3"]  # Replace with your joint names
            # joint_state.position = [0.0, 0.0, 0.0]  # Replace with your joint positions

            names, positions = self.hello_robot.get_joints()
            joint_state.name = names
            joint_state.position = positions

            self.pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass