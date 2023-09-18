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

import time
import rospy
import cv2
import numpy as np
import sys
import PyKDL
from PIL import Image

from multiprocessing import Process


def publisher_process(robot):
    try:
        rospy.init_node('publisher')
    except:
        print('node already initialized publisher')

    publisher = JointStatePublisher(robot)
    publisher.publish()

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

    if args.mode == "capture" or args.mode == "pick":
        global_parameters.INIT_WRIST_PITCH = -1.57

    # Joint state publisher
    # pub_proc = Process(target = publisher_process, args=(hello_robot, ))
    # pub_proc.start()

    try:
        rospy.init_node('hello_robot_node')
    except:
        print('node already initialized hello_robot')

    # Moving robot to intital position
    print(INIT_ARM_POS, INIT_WRIST_PITCH, INIT_WRIST_ROLL, INIT_WRIST_YAW, gripper_pos)
    hello_robot.move_to_position(arm_pos=INIT_ARM_POS,
                                wrist_pitch = INIT_WRIST_PITCH,
                                wrist_roll = INIT_WRIST_ROLL,
                                wrist_yaw = INIT_WRIST_YAW,
                                gripper_pos = gripper_pos)
    time.sleep(1)

    hello_robot.move_to_position(lift_pos=INIT_LIFT_POS,
                                wrist_pitch = global_parameters.INIT_WRIST_PITCH,
                                wrist_roll = INIT_WRIST_ROLL,
                                wrist_yaw = INIT_WRIST_YAW)

    time.sleep(1)

    # Intialsiing Camera
    #if args.mode == "move" or args.mode == "capture":
    #camera = RealSenseCamera()
    camera = RealSenseCamera(hello_robot.robot)

    if args.mode == "capture":
        image_publisher = ImagePublisher(camera)
        image_publisher.publish_image()
        exit()

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

    if args.mode == "pick":
        image_publisher = ImagePublisher(camera)
        translation, rotation, depth = image_publisher.publish_image()
        point = PyKDL.Vector(-translation[1], -translation[0], translation[2])
        #point = PyKDL.Vector(translation[1], -translation[0], translation[2])
        
        # Rotation from model frame to pose frame
        rotation1 = PyKDL.Rotation(rotation[0][0], rotation[0][1], rotation[0][2],
                                   rotation[1][0],  rotation[1][1], rotation[1][2],
                                    rotation[2][0],  rotation[2][1], rotation[2][2])

        # Rotation from camera frame to model frame
        rotation1_bottom = PyKDL.Rotation(0.0000000, -1.0000000,  0.0000000,
                                    -1.0000000,  0.0000000,  0.0000000, 
                                    0.0000000,  0.0000000, 1.0000000)
        print(f"Points frame rotation - {rotation1.GetRPY()}, {rotation1.GetEulerZYX()}")
        print(rotation1)

        # Rotation from camera frame to pose frame
        rotation =  rotation1_bottom * rotation1
        print(rotation)
        print(f"Camera frame rotation - {rotation.GetRPY()}, {rotation.GetEulerZYX()}")

    dest_frame = PyKDL.Frame(rotation, point) 
    
    # Camera frame to gripper frame transformation
    if args.transform and transform_node is not None:

        # transform - Rotation and translation from camera frame to gripper frame
        transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)

        print('dest_frame', dest_frame.p)
        # Rotation from gripper frame frame to gripper frame
        transformed_frame = transform * dest_frame

        if transform_node == GRIPPER_MID_NODE and args.mode == "move":
            transformed_frame.p[2] -= 0.2
    else:
        transformed_point = point
    
    print(transformed_frame.p)
    print(transformed_frame.M.GetRPY())

    if args.mode == "move":
        hello_robot.move_to_pose(
            [transformed_frame.p[0], transformed_frame.p[1], transformed_frame.p[2]],
            [0, 0, 0],
            # [-transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[2]+1.53],
            [gripper_pos]
        )
    elif args.mode == "pick":

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
        time.sleep(4)

        # Calculating new co-rodinates of pose center
        if args.transform and transform_node is not None:
            transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
            transformed_point1 = transform * point

            transformed_point1[2] -= (0.195 - depth)
        
        # print()
        # Moving gripper to pose center
        hello_robot.move_to_pose(
            [transformed_point1.x(), transformed_point1.y(), transformed_point1.z() - 0.2],
            [0, 0, 0],
            # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
            [gripper_pos]
        )
        # exit()
        # hello_robot.move_to_pose(
        #     [0, 0, 0.05],
        #     [0, 0, 0],
        #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        #     [gripper_pos]
        # )
        print("1\n\n\n\n\n\n\n\n\n\n\n")
        hello_robot.move_to_pose(
            [0, 0, 0.18],
            [0, 0, 0],
            # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
            [gripper_pos]
        )
        print("2\n\n\n\n\n\n\n\n\n\n\n")
        # hello_robot.move_to_pose(
        #     [0, 0, 0.05],
        #     [0, 0, 0],
        #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        #     [gripper_pos],
        #     1
        # )
        hello_robot.move_to_pose(
            [0, 0, 0.02],
            [0, 0, 0],
            # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
            [gripper_pos]
        )
        print("3\n\n\n\n\n\n\n\n\n\n\n")
        # Picking the object
        # if (args.mode == "pick"):
        hello_robot.pickup(abs(0))
        # Put it down for now
        time.sleep(3)
        hello_robot.move_to_position(gripper_pos = 1)
        hello_robot.move_to_position(arm_pos = INIT_ARM_POS)
