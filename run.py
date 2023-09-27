import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

#from home_robot_hw.remote import StretchClient
from robot import HelloRobot
#import stretch_body.robot
import zmq
import time

from global_parameters import *
import global_parameters
from args import get_args
from camera import RealSenseCamera
#from segment import segment_image
from utils import potrait_to_landscape, segment_point_cloud, plane_detection, display_image_and_point
from nodes import JointStatePublisher, Listener, ImagePublisher

import rospy
import cv2
import numpy as np
import sys
import PyKDL
from PIL import Image

from multiprocessing import Process


#robot = StretchClient()
#robot.switch_to_navigation_mode()
#robot.move_to_nav_posture()
#robot = stretch_body.robot.Robot()
#robot.startup()

POS_TOL = 0.1
YAW_TOL = 0.2

X_OFFSET = -6.325
Y_OFFSET = -1.069
THETA_OFFSET =  4.059

def navigate(robot, xyt_goal):
    while True:
        robot.nav.navigate_to(xyt_goal)
        xyt_curr = robot.nav.get_base_pose()
        print("The robot currently loactes at " + str(xyt_curr))
        time.sleep(0.5)
        if np.allclose(xyt_curr[:2], xyt_goal[:2], atol=POS_TOL) and np.allclose(xyt_curr[2], xyt_goal[2], atol=YAW_TOL):
            print("The robot is finally at " + str(xyt_goal))
            break

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    paths = data.data
    i = 0
    while i < len(paths):
        x = -paths[i]
        y = paths[i + 1]
        navigate(robot, np.array([x, y, 0]))
        i += 2

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    A = np.array(A)
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(np.ascontiguousarray(A), flags, copy=copy, track=track)

# use zmq to receive a numpy array
def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    A = np.frombuffer(msg, dtype=md['dtype'])
    return A.reshape(md['shape'])

def run_navigation(robot, socket):
    # Reset robot
    #print("Resetting robot...")
    #robot.reset()
    start_xy = robot.nav.get_base_pose()
    start_xy[0] += X_OFFSET
    start_xy[1] += Y_OFFSET
    start_xy[2] += THETA_OFFSET
    A = str(input("Enter A: "))
    print("A = ", A)
    B = str(input("Enter B: "))
    print("B = ", B)
    send_array(socket, start_xy)
    print(socket.recv_string())
    socket.send_string(A)
    print(socket.recv_string())
    socket.send_string(B)
    print(socket.recv_string())
    socket.send_string("Waiting for path")
    paths = recv_array(socket)
    print(paths)
    for path in paths:
        path = (path[0] - X_OFFSET, path[1] - Y_OFFSET, path[2] - THETA_OFFSET)
        print(path)
    for path in paths:
        path = (path[0] - X_OFFSET, path[1] - Y_OFFSET, path[2] - THETA_OFFSET)
        print(path)
        #navigate(robot, path)
    xyt = robot.nav.get_base_pose()
    xyt[2] = xyt[2] + np.pi / 2
    navigate(robot, xyt)
    return A

def run_manipulation(args, hello_robot, socket, text, image_publisher, transform_node, base_node):
    
    gripper_pos = 1
    global_parameters.INIT_WRIST_PITCH = -1.57

    translation, rotation, depth = image_publisher.publish_image(text)
    point = PyKDL.Vector(-translation[1], -translation[0], translation[2])
        
    rotation1 = PyKDL.Rotation(rotation[0][0], rotation[0][1], rotation[0][2],
                                rotation[1][0],  rotation[1][1], rotation[1][2],
                                rotation[2][0],  rotation[2][1], rotation[2][2])

    # Rotation from camera frame to model frame
    rotation1_bottom = PyKDL.Rotation(0.0000000, -1.0000000,  0.0000000,
                                    -1.0000000,  0.0000000,  0.0000000, 
                                    0.0000000,  0.0000000, 1.0000000)
    print(f"Points frame rotation - {rotation1.GetRPY()}, {rotation1.GetEulerZYX()}")

    # Rotation from camera frame to pose frame
    rotation =  rotation1_bottom * rotation1
    print(f"Camera frame rotation - {rotation.GetRPY()}, {rotation.GetEulerZYX()}")

    dest_frame = PyKDL.Frame(rotation, point) 

    # transform - Rotation and translation from camera frame to gripper frame
    transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
    transformed_frame = transform * dest_frame

    print("pose: ", transformed_frame.p)
    print("rotation: ", transformed_frame.M.GetRPY())

    # Rotation for aligning gripper frame to model pose frame
    rotation2_top = PyKDL.Rotation(0, 0, 1, 1, 0, 0, 0, -1, 0)
        
    # final Rotation of gripper to hold the objet
    final_rotation = transformed_frame.M * rotation2_top

    # Only rotating the gripper 
    hello_robot.move_to_pose(
        [0, 0, 0],
        [final_rotation.GetRPY()[0], final_rotation.GetRPY()[1], final_rotation.GetRPY()[2]],
        [gripper_pos],
    )
    time.sleep(1)

    # Calculating new co-rodinates of pose center
    if args.transform and transform_node is not None:
        transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        transformed_point1 = transform * point
        transformed_point1[2] -= (0.195 - depth)
        
    # Moving gripper to pose center
    hello_robot.move_to_pose(
        [transformed_point1.x(), transformed_point1.y(), transformed_point1.z() - 0.2],
        [0, 0, 0],
        [gripper_pos]
    )
    print(hello_robot.robot._ros_client.get_joint_state()[0])
    print("\n\n\n\n\n")
    hello_robot.move_to_pose(
        [0, 0, 0.18],
        [0, 0, 0],
        [gripper_pos]
    )
    print(hello_robot.robot._ros_client.get_joint_state()[0])
    print("\n\n\n\n\n")
    hello_robot.move_to_pose(
        [0, 0, 0.02],
        [0, 0, 0],
        [gripper_pos]
    )
    print(hello_robot.robot._ros_client.get_joint_state()[0])
    print("\n\n\n\n\n")
    hello_robot.pickup(abs(0))
    # Put it down for now
    time.sleep(3)
    hello_robot.move_to_position(gripper_pos = 1)
    hello_robot.move_to_position(arm_pos = INIT_ARM_POS)


def run():
    hello_robot = HelloRobot()
    args = get_args()
    
    if args.base_frame  == "gripper_camera":
        base_node = CAMERA_NODE
    elif args.base_frame == "top_camera":
        base_node = TOP_CAMERA_NODE
    elif args.base_frame == "gripper_fingertip_left":
        base_node = GRIPPER_FINGERTIP_LEFT_NODE
    elif args.base_frame == "gripper_fingertip_right":
        base_node = GRIPPER_FINGERTIP_RIGHT_NODE

    if args.transform_node == "gripper_fingertip_left":
        transform_node = GRIPPER_FINGERTIP_LEFT_NODE
    elif args.transform_node == "gripper_fingertip_right":
        transform_node = GRIPPER_FINGERTIP_RIGHT_NODE
    elif args.transform_node == "gripper_left":
        transform_node = GRIPPER_FINGER_LEFT_NODE
    elif args.transform_node == "gripper_mid":
        transform_node = GRIPPER_MID_NODE
    if (args.transform):
        hello_robot = HelloRobot(end_link=transform_node)
    else:
        hello_robot = HelloRobot(end_link=base_node)

    camera = RealSenseCamera(hello_robot.robot)
    image_publisher = ImagePublisher(camera)

    context = zmq.Context()
    nav_socket = context.socket(zmq.REQ)
    nav_socket.connect("tcp://172.24.71.253:5555")
    manip_socket = context.socket(zmq.REQ)
    manip_socket.connect("tcp://172.24.71.253:5556")

    while True:
        hello_robot.robot.switch_to_navigation_mode()
        hello_robot.robot.move_to_post_nav_posture()
        hello_robot.robot.head.look_front()
        A = run_navigation(hello_robot.robot, nav_socket)
        #A = "yellow bottle"
        hello_robot.robot.switch_to_manipulation_mode()
        hello_robot.robot.move_to_manip_posture()
        hello_robot.robot.head.look_at_ee()
        run_manipulation(args, hello_robot, manip_socket, A, image_publisher, transform_node, base_node)

if __name__ == '__main__':
    run()
