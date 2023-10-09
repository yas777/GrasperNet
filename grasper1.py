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
import math

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
    # exit()
    # Intialsiing Camera
    #if args.mode == "move" or args.mode == "capture":
    # camera = RealSenseCamera()

    camera = RealSenseCamera(hello_robot.robot)
    if args.mode == "capture":
        image_publisher = ImagePublisher(camera)
        image_publisher.publish_image(args.picking_object)
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

        # Centering the object
        # args.picking_object = 'green bottle'
        base_trans, head_tilt, _, _ = image_publisher.publish_image(args.picking_object, head_tilt=INIT_HEAD_TILT)
        # head_pan = INIT_HEAD_PAN 
        head_pan = INIT_HEAD_PAN
        # head_tilt = INIT_HEAD_TILT
        head_tilt = INIT_HEAD_TILT + (head_tilt)
        print(f"head_tilt - {head_tilt}")
        hello_robot.move_to_position(base_trans=base_trans[0],
                                    head_pan=head_pan,
                                    head_tilt=head_tilt)

        time.sleep(0.7)
        # Getting final translation, rotation of gripper
        translation, rotation, depth, cropped = image_publisher.publish_image(args.picking_object, head_tilt=head_tilt)
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
        
        # 
        gripper_yaw = math.atan(rotation[1][0]/rotation[0][0])
        print(f"gripper_yaw - {gripper_yaw}")


        # Remove yaw from gripper rotation as we are rotating base
        # rotation1_yaw = PyKDL.Rotation(math.cos(gripper_yaw), math.sin(gripper_yaw), 0,
        #                                 -math.sin(gripper_yaw), math.cos(gripper_yaw), 0,
        #                                 0, 0, 1) 
        
        print(f"Points frame rotation - {rotation1.GetRPY()}, {rotation1.GetEulerZYX()}")
        print(rotation1)

        # Rotation from camera frame to pose frame
        rotation =  rotation1_bottom * rotation1
        print(rotation)
        print(f"Camera frame rotation - {rotation.GetRPY()}, {rotation.GetEulerZYX()}")

    dest_frame = PyKDL.Frame(rotation, point) 
    
    # Camera frame to gripper frame transformation
    hello_robot.move_to_position(lift_pos = 1.0, head_pan = None, head_tilt = None)
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
    
    print("pose: ", transformed_frame.p)
    print("rotation: ", transformed_frame.M.GetRPY())

    #exit()
    if args.mode == "move":
        hello_robot.move_to_pose(
            [transformed_frame.p[0], transformed_frame.p[1], transformed_frame.p[2]],
            [0, 0, 0],
            # [-transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[2]+1.53],
            [gripper_pos],
            move_mode=1
        )
    elif args.mode == "pick":

        # Rotation for aligning gripper frame to model pose frame
        rotation2_top = PyKDL.Rotation(0, 0, 1, 1, 0, 0, 0, -1, 0)
        
        # final Rotation of gripper to hold the objet
        final_rotation = transformed_frame.M * rotation2_top
        
        # Rotating the base to align the gripper with object
        # hello_robot.robot.switch_to_navigation_mode()
        # hello_robot.move_to_position(base_theta=gripper_yaw)

        # hello_robot.robot.switch_to_manipulation_mode()
        # Only rotating the gripper 
        # time.sleep(1)
        hello_robot.move_to_pose(
            [0, 0, 0],
            [final_rotation.GetRPY()[0], final_rotation.GetRPY()[1], final_rotation.GetRPY()[2]],
            [gripper_pos],
        )

        # Calculating new co-rodinates of pose center
        if args.transform and transform_node is not None:
            transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
            transformed_point1 = transform * point
            
            print(f"transformed point1 : {transformed_point1}")
            # if depth < 0.01 or cropped:
            #     print("depth first loop")
            #     transformed_point1[2] -= (0.20 - depth)
            #     ref_diff = 0.245 - depth
            # else:
            transformed_point1[2] -= (0.185)
            ref_diff = 0.185
            print("depth second loop")
            print(f"transformed point1 : {transformed_point1}")
        # print()
        
        # Moving gripper to pose center
        hello_robot.move_to_pose(
            [transformed_point1.x(), transformed_point1.y(), transformed_point1.z() - 0.2],
            [0, 0, 0],
            # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
            [gripper_pos],
            move_mode = 1
        )

        time.sleep(1)
        transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        transformed_point2 = transform * point
        print(f"transformed point2 : {transform * point}")
        curr_diff = transformed_point2.z()

        diff = abs(curr_diff - ref_diff)
        velocities = [1]*8
        velocities[5:] = [0.007, 0.007, 0.007, 0.007]
        velocities[0] = 0.005
        if diff > 0.08:
            dist = diff - 0.08
            hello_robot.move_to_pose(
                [0, 0, dist],
                [0, 0, 0],
                # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
                [gripper_pos]
            )
            time.sleep(1)
            transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
            print(f"transformed point3 : {transform * point}")
            diff = diff - dist
            
        while diff > 0.01:
            dist = min(0.03, diff)
            hello_robot.move_to_pose(
                [0, 0, dist],   
                [0, 0, 0],
                # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
                [gripper_pos],
                velocities=velocities
            )
            time.sleep(1)
            transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
            print(f"transformed point3 : {transform * point}")
            diff = diff - dist



        # exit()
        # hello_robot.move_to_pose(
        #     [0, 0, 0.05],
        #     [0, 0, 0],
        #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        #     [gripper_pos]
        # )
        # print(hello_robot.robot._ros_client.get_joint_state()[0])
        # print("\n\n\n\n\n\n\n\n\n\n\n")
        # hello_robot.move_to_pose(
        #      [0, 0, 0.12],
        #      [0, 0, 0],
        #      # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        #     [gripper_pos]
        # )
        # time.sleep(1)
        # transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        # print(f"transformed point2 : {transform * point}")

        # print(hello_robot.robot._ros_client.get_joint_state()[0])
        # print("\n\n\n\n\n\n\n\n\n\n\n")
        # hello_robot.move_to_pose(
        #     [0, 0, 0.05],
        #     [0, 0, 0],
        #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        #     [gripper_pos],
        #     1
        # )
        # velocities = [1]*8
        # velocities[4:] = [0.005, 0.005, 0.005, 0.005]
        # velocities[0] = 0.01
        # # velocities[0] = 0.01
        # hello_robot.move_to_pose(
        #     [0, 0, 0.08],   
        #     [0, 0, 0],
        #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        #     [gripper_pos],
        #     velocities=velocities
        # )
        # time.sleep(1)
        # transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        # print(f"transformed point2 : {transform * point}")
        # print(hello_robot.robot._ros_client.get_joint_state()[0])
        # print("\n\n\n\n\n\n\n\n\n\n\n")
        # Picking the object
        # if (args.mode == "pick"):
        hello_robot.pickup(abs(0))
        # Put it down for now
        time.sleep(3)
        hello_robot.move_to_position(gripper_pos = 1)
        hello_robot.move_to_position(arm_pos = INIT_ARM_POS)
