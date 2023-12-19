from robot import HelloRobot
from camera import DemoApp
from camera import CaptureImage
from global_parameters import *
import global_parameters
from args import get_args
from camera import RealSenseCamera
#from segment import segment_image
from utils import potrait_to_landscape, segment_point_cloud, plane_detection, display_image_and_point
from nodes import JointStatePublisher, Listener, ImagePublisher
from utils.grasper_utils import pickup, move_to_point

import zmq
import time
import rospy
import cv2
import numpy as np
import sys
import PyKDL
from PIL import Image
import math

from multiprocessing import Process


def publisher_process(robot):
    try:
        rospy.init_node('publisher')
    except:
        print('node already initialized publisher')

    publisher = JointStatePublisher(robot)
    publisher.publish()

def capture_and_process_image(camera, args, socket, hello_robot, INIT_HEAD_TILT, top_down = False):
    print(args.mode)
    # point selector
    if args.mode == "move":
        # Image Capturing 
        rgb_image, depth_image, points = camera.capture_image()
        h, _, _ = rgb_image.shape

        # Displaying windows for point selection
        ix, iy = camera.visualize_image()
        print(f"ix - {ix},iy - {iy}")

        # Image to world co-ordinates conversion
        sx, sy, sz = camera.pixel2d_to_point3d(ix, iy)
        point = PyKDL.Vector(sx, -sy, sz)
        #point = PyKDL.Vector(-sy, sx, sz)
        print(f"x - {sx}, y - {sy}, z - {sz}")

        rotation = PyKDL.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1)
    
        return rotation, point

    print(args.picking_object)
    if args.mode == "pick" or args.mode == "place":
        print("hello")
        if args.mode == "pick":
            obj = args.picking_object
        else:
            obj = args.placing_object

        image_publisher = ImagePublisher(camera, socket)

        # Centering the object
        head_tilt_angles = [0, -0.1, 0.1]
        tilt_retries, side_retries = 1, 0
        retry_flag = True
        head_tilt = INIT_HEAD_TILT
        head_pan = INIT_HEAD_PAN

        while(retry_flag):
            translation, rotation, depth, cropped, retry_flag = image_publisher.publish_image(obj, args.mode, head_tilt=head_tilt, top_down = top_down)

            print(f"retry flag : {retry_flag}")
            if (retry_flag == 1):
                base_trans = translation[0]
                head_tilt += (rotation[0])

                hello_robot.move_to_position(base_trans=base_trans,
                                        head_pan=head_pan,
                                        head_tilt=head_tilt)
                time.sleep(4)
            
            elif (side_retries == 2 and tilt_retries == 3):
                hello_robot.move_to_position(base_trans=0.1, head_tilt=head_tilt)
                side_retries = 3

            elif retry_flag == 2:
                if (tilt_retries == 3):
                    if (side_retries == 0):
                        hello_robot.move_to_position(base_trans=0.1, head_tilt=head_tilt_angles[0])
                        side_retries = 1
                    else:
                        hello_robot.move_to_position(base_trans=-0.2, head_tilt=head_tilt_angles[0])
                        side_retries = 2
                    tilt_retries = 1
                else:
                    print(f"retrying with head tilt : {head_tilt + head_tilt_angles[tilt_retries]}")
                    hello_robot.move_to_position(head_pan=head_pan,
                                            head_tilt=head_tilt + head_tilt_angles[tilt_retries])
                    tilt_retries += 1
                    time.sleep(1)
            
            elif side_retries == 3:
                print("No poses found in all retries")
                time.sleep(2)
            

        if args.mode == "place":
            translation = PyKDL.Vector(-translation[1], -translation[0], -translation[2])

        return rotation, translation, depth


if __name__ == "__main__":
    args = get_args()

    # Initalize robot and move to a height of 0.86
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
    
    if args.mode == "pick":
        gripper_pos = 1
    else:
        gripper_pos = 0

    if args.mode == "capture" or args.mode == "pick" or args.mode == "place":
        global_parameters.INIT_WRIST_PITCH = -1.57

    # Joint state publisher
    # pub_proc = Process(target = publisher_process, args=(hello_robot, ))
    # pub_proc.start()

    try:
        rospy.init_node('hello_robot_node')
    except:
        print('node already initialized hello_robot')

    # Moving robot to intital position

    print(args.picking_object)
    print(INIT_ARM_POS, INIT_WRIST_PITCH, INIT_WRIST_ROLL, INIT_WRIST_YAW, gripper_pos)
    hello_robot.move_to_position(arm_pos=INIT_ARM_POS,
                                head_pan=INIT_HEAD_PAN,
                                head_tilt=INIT_HEAD_TILT,
                                gripper_pos = gripper_pos)
    time.sleep(1)
    
    hello_robot.move_to_position(lift_pos=INIT_LIFT_POS,
                                wrist_pitch = global_parameters.INIT_WRIST_PITCH,
                                wrist_roll = INIT_WRIST_ROLL,
                                wrist_yaw = INIT_WRIST_YAW)
    time.sleep(1)

    # transform, _, _ = hello_robot.get_joint_transform("base_link", GRIPPER_MID_NODE)
    # print(transform)
    # exit()
    # Intialsiing Camera
    #if args.mode == "move" or args.mode == "capture":
    # camera = RealSenseCamera()

    camera = RealSenseCamera(hello_robot.robot)

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://100.107.224.62:5556")

    retry = True
    while retry:
        rotation, translation, depth = capture_and_process_image(camera, args, socket, hello_robot)

        if args.mode == "move":
            move_to_point(hello_robot, translation, base_node, transform_node)
            retry = False
        elif args.mode == "pick":
            pickup(hello_robot, rotation, translation, base_node, transform_node, gripper_depth = depth)
            args.mode = "place"
        else:
            hello_robot.move_to_position(lift_pos=1)
            hello_robot.move_to_position(wrist_pitch=0)
            move_to_point(hello_robot, translation, base_node, transform_node)
            hello_robot.move_to_position(gripper_pos=1)
            retry = False

        
