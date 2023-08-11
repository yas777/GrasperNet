from robot import HelloRobot
from camera import DemoApp
from camera import CaptureImage
from global_parameters import *
import global_parameters
from args import get_args
from camera import RealSenseCamera
from segment import segment_image
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

    if args.mode == "capture":
        global_parameters.INIT_WRIST_PITCH = -1.57

    # Joint state publisher
    # pub_proc = Process(target = publisher_process, args=(hello_robot, ))
    # pub_proc.start()

    try:
        rospy.init_node('hello_robot_node')
    except:
        print('node already initialized hello_robot')

    # Moving robot to intital position
    hello_robot.move_to_position(arm_pos=INIT_ARM_POS,
                                wrist_pitch = INIT_WRIST_PITCH,
                                wrist_roll = INIT_WRIST_ROLL,
                                wrist_yaw = INIT_WRIST_YAW,
                                gripper_pos = gripper_pos)
    time.sleep(4)

    hello_robot.move_to_position(lift_pos=INIT_LIFT_POS,
                                wrist_pitch = global_parameters.INIT_WRIST_PITCH,
                                wrist_roll = INIT_WRIST_ROLL,
                                wrist_yaw = INIT_WRIST_YAW)

    time.sleep(2)

    # Intialsiing Camera
    if args.mode == "move" or args.mode == "capture":
        camera = RealSenseCamera()

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
        print(f"x - {sx}, y - {sy}, z - {sz}")

        rotation = PyKDL.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1)

    if args.mode == "pick":
        # point = PyKDL.Vector(0.04464857, 0.02098933, 0.75800002)
        point = PyKDL.Vector(-0.23317847, 0.28965002, 0.80186498)

        # rotation1 = PyKDL.Rotation(2.39388689e-01, -9.55172122e-01, -1.74181908e-01, 
        #                             -4.23444882e-02,  1.68956459e-01, -9.84713495e-01,
        #                             9.70000029e-01,  2.43104920e-01, -1.06264535e-08)
        
        # rotation1 = PyKDL.Rotation(-3.11710656e-01,  8.86093080e-01,  3.43038589e-01,
        #                             1.13836229e-01, -3.23599726e-01,  9.39321280e-01, 
        #                             9.43333328e-01,  3.31846684e-01, -1.45054795e-08)
        
        # rotation1 = PyKDL.Rotation(0.07559296, -0.96571505, -0.24835478, 
        #                              0.28120205, 0.25960431, -0.9238674,
        #                               0.95666665, 0, 0.29118532)

        # rotation1 = PyKDL.Rotation(0.02277477, -0.9989326,  -0.04018656,
        #                             0.49252543,  0.04619145, -0.86907136,
        #                             0.87,        0,          0.49305171)

        rotation1 = PyKDL.Rotation(-0.11492483,  0.99317348, -0.01996705,
                                    0.36639848,  0.06106357,  0.92845213,
                                    0.92333335,  0.09938631, -0.370915  )
        rotation1_top = PyKDL.Rotation(-0.0000000, -1.0000000,  0.0000000,
                                    -1.0000000,  0.0000000,  0.0000000, 
                                    0.0000000,  0.0000000, -1.0000000)
        rotation1_bottom = PyKDL.Rotation(0.0000000, -1.0000000,  0.0000000,
                                    -1.0000000,  0.0000000,  0.0000000, 
                                    0.0000000,  0.0000000, 1.0000000)
        rotation2_top = PyKDL.Rotation(-1, 0, 0, 0, 0, 1, 0, 1, 0)
        # rotation = PyKDL.Rotation(0.96571505,   -0.07559296, -0.24835478, 
        #                         -0.25960431,    -0.28120205, -0.9238674,
        #                         -0,             -0.95666665, 0.29118532)
        print(f"Points frame rotation - {rotation1.GetRPY()}, {rotation1.GetEulerZYX()}")
        print(rotation1)
        # rotation = PyKDL.Rotation.RPY(rotation.GetRPY()[1], rotation.GetRPY()[0], rotation.GetRPY()[2]-1.53)
        rotation =  rotation1_bottom * rotation1
        print(rotation)
        print(f"Camera frame rotation - {rotation.GetRPY()}, {rotation.GetEulerZYX()}")

    dest_frame = PyKDL.Frame(rotation, point) 
    
    # Camera frame to gripper frame transformation
    if args.transform and transform_node is not None:
        transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        transformed_frame = transform * dest_frame

        if transform_node == GRIPPER_MID_NODE and args.mode == "move":
            transformed_frame.p[2] -= 0.22

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
        rotation2_top = PyKDL.Rotation(0, 0, 1, 1, 0, 0, 0, -1, 0)
        
        final_rotation = transformed_frame.M * rotation2_top
        hello_robot.move_to_pose(
            [0, 0, 0],
            [final_rotation.GetRPY()[0], final_rotation.GetRPY()[1], final_rotation.GetRPY()[2]],
            [gripper_pos],
        )
        time.sleep(4)

        if args.transform and transform_node is not None:
            transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
            transformed_point1 = transform * point

            transformed_point1[2] -= 0.185
        
        hello_robot.move_to_pose(
            [transformed_point1.x(), transformed_point1.y(), transformed_point1.z()],
            [0, 0, 0],
            # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
            [gripper_pos]
        )
        time.sleep(3)

        # Picking the object
        if (args.mode == "pick"):
            hello_robot.pickup(abs(0))
        
    exit()

    # exit()
    # print(point, transformed_point, transformed_point)
    # print(rotation.GetRPY())
    
    # Moving robot to a desired point

    # hello_robot.move_to_position(
    #     wrist_pitch = 0,
    #     wrist_roll = 0,
    #     wrist_yaw = 0
    # )

    # time.sleep(5) 

    

    
    rotation3_top = PyKDL.Rotation(0.7660444, -0.6427876,  0.0000000, 0.6427876,  0.7660444,  0.0000000, 0.0000000,  0.0000000,  1.0000000)
    rotation2_bottom = PyKDL.Rotation(0, -1, 0, -1, 0, 0, 0, 0, 1)

    # final_rotation =   rotation2_top * transformed_frame.M
    final_rotation = transformed_frame.M * rotation2_top
    print(f"final gripper rotation - {final_rotation.GetRPY()}, {final_rotation.GetRPY()}")
    print(final_rotation)
    hello_robot.move_to_pose(
        # [transformed_point.x(), transformed_point.y(), transformed_point.z()+0.03],
        [0, 0, 0],
        # [0, 1, 0],
        # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        # [0, 0, 0],
        [final_rotation.GetRPY()[0], final_rotation.GetRPY()[1], final_rotation.GetRPY()[2]],
        # [transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[2]],
        [gripper_pos],
    )

    time.sleep(4)

    # hello_robot.move_to_pose(
    #     # [transformed_point.x(), transformed_point.y(), transformed_point.z()+0.03],
    #     [0, 0, 0],
    #     # [0, 1, 0],
    #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
    #     # [0, 0, 0],
    #     [0, 0, 1.53],
    #     # [transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[2]],
    #     [gripper_pos]
    # )

    # exit()
    

    # hello_robot.move_to_pose(
    #     # [transformed_point.x(), transformed_point.y(), transformed_point.z()+0.03],
    #     [0, 0, 0],
    #     # [0, 1, 0],
    #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
    #     [-transformed_frame.M.GetRPY()[1], -transformed_frame.M.GetRPY()[0], -transformed_frame.M.GetRPY()[2]],
    #     # [-transformed_frame.M.GetRPY()[1], -transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[2]+1.53],
    #     [gripper_pos]
    # )

    # time.sleep(5)

    if args.transform and transform_node is not None:
        transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        transformed_point1 = transform * point

        transformed_point1[2] -= 0.185

    # origin = PyKDL.Vector(0, 0, 0)
    # transformed_frame1 = PyKDL.Frame(rotation, origin)
    # transformed_point1 = transformed_frame1.Inverse() * transformed_point
    print(transformed_point1)

    hello_robot.move_to_pose(
        [transformed_point1.x(), transformed_point1.y(), transformed_point1.z()],
        [0, 0, 0],
        # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        [gripper_pos]
    )
    
    # Picking the object
    if (args.mode == "pick"):
        hello_robot.pickup(abs(0))

    # pub_proc.terminate()
    # pub_proc.join()
    # final_rel_pos = PyKDL.Vector(0.5, 0, 0)
    # hello_robot.move_to_pose(final_rel_pos, [0,0,0], [0.5])