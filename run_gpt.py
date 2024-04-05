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
from utils import potrait_to_landscape, segment_point_cloud, plane_detection, display_image_and_point

import rospy
import cv2
import numpy as np
import sys
import PyKDL
from PIL import Image

from multiprocessing import Process

from utils.grasper_utils import pickup, move_to_point
from grasper import capture_and_process_image
from openai_client import OpenaiClient
from run import load_offset, navigate, callback, send_array, recv_array, run_navigation, run_manipulation, run_place, compute_tilt


POS_TOL = 0.1
YAW_TOL = 0.2

def run():
    args = get_args()
    load_offset(args.x1, args.y1, args.x2, args.y2)
    
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
        hello_robot = HelloRobot(end_link=transform_node, stretch_client_urdf_file='home-robot/assets/hab_stretch/urdf')
    else:
        hello_robot = HelloRobot(end_link=base_node, stretch_client_urdf_file='home-robot/assets/hab_stretch/urdf')

    global_parameters.INIT_WRIST_PITCH = -1.57
    #camera = RealSenseCamera(hello_robot.robot)
    #image_publisher = ImagePublisher(camera)

    context = zmq.Context()
    nav_socket = context.socket(zmq.REQ)
    nav_socket.connect("tcp://" + args.ip + ":" + str(args.navigation_port))
    #nav_socket.connect("tcp://172.24.71.253:5555")
    anygrasp_socket = context.socket(zmq.REQ)
    # args.manipulation_port = 5556
    anygrasp_socket.connect("tcp://" + args.ip + ":" + str(args.manipulation_port + 0))
    anygrasp_open_socket = context.socket(zmq.REQ)
    anygrasp_open_socket.connect("tcp://" + args.ip + ":" + str(args.manipulation_port + 1))
    topdown_socket = context.socket(zmq.REQ)
    topdown_socket.connect("tcp://" + args.ip + ":" + str(args.manipulation_port + 2))


    client = None
    debug = True
    i = 0

    while True:

        if debug:
            # print("Pick object")
            # A, B = read_input()
            # print("Place location")
            # C, D = read_input()
            if i == 0:
                A, B = "green bottle", ""
                C, D = "sink", ""
            elif i == 1:
                A, B  = "green cup", ""
                C, D = "sink", ""
            elif i == 2:
                A, B = "blue mug", ""
                # A, B = "blue mug", ""
                C, D = "sink", ""

            # Another case
            # i = 2
            # A, B  = "green and white plush cactus", "table"
            # A, B = "cactus plushie", "table"
            # C, D = "baby mobile and carrier", ""
        else:
            if client is None:
                # Creating openai client for demos
                client = OpenaiClient(use_specific_objects=False)

        #if input("You want to run navigation? Y or N") != "N":
        if True:
            # A, B = read_input()

            hello_robot.robot.switch_to_navigation_mode()
            hello_robot.robot.move_to_post_nav_posture()
            hello_robot.robot.head.look_front()
            print(A, B)
            end_xyz = run_navigation(hello_robot.robot, nav_socket, A, B)
            if not end_xyz is None:
                camera_xyz = hello_robot.robot.head.get_pose()[:3, 3]
                INIT_HEAD_TILT = compute_tilt(camera_xyz, end_xyz)

        print('debug coordinates', hello_robot.robot.nav.get_base_pose())
        # if input("You want to run manipulation? Y or N ") != 'N':
        if True:
            #if (A is None):
            #    A, _ = read_input()
            hello_robot.robot.switch_to_manipulation_mode()
            hello_robot.robot.head.look_at_ee()
            run_manipulation(args, hello_robot, anygrasp_socket, A, transform_node, base_node)
        
        print('debug coordinates', hello_robot.robot.nav.get_base_pose())
        # if input("You want to run navigation? Y or N") != "N":
        if True:
            A, B = C, D  # read_input()

            hello_robot.robot.switch_to_navigation_mode()
            # hello_robot.robot.move_to_post_nav_posture()
            hello_robot.robot.head.look_front()
            end_xyz = run_navigation(hello_robot.robot, nav_socket, A, B)
            if not end_xyz is None:
                camera_xyz = hello_robot.robot.head.get_pose()[:3, 3]
                INIT_HEAD_TILT = compute_tilt(camera_xyz, end_xyz)

        #if input("You want to run place? Y or N") != 'N':
        if True:
            #if (A is None):
            #    A, _ = read_input()
            hello_robot.robot.switch_to_manipulation_mode()
            hello_robot.robot.head.look_at_ee()
            run_place(args, hello_robot, anygrasp_socket, A, transform_node, base_node)
        time.sleep(1)

        i += 1
        if i > 2: break

if __name__ == '__main__':
    run()
