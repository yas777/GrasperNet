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
from .run import load_offset, navigate, callback, send_array, recv_array


POS_TOL = 0.1
YAW_TOL = 0.2

X_OFFSET, Y_OFFSET, THETA_OFFSET, r2n_matrix, n2r_matrix = None, None, None, None, None


def run_navigation(robot, socket, A, B):
    # Reset robot
    #print("Resetting robot...")
    #robot.reset()
    start_xy = robot.nav.get_base_pose()
    print("start =", start_xy)
    #start_xy[0] += X_OFFSET
    #start_xy[1] += Y_OFFSET
    transformed_start_xy = r2n_matrix @ np.array([start_xy[0], start_xy[1], 1])
    start_xy[0], start_xy[1] = transformed_start_xy[0], transformed_start_xy[1]
    start_xy[2] += THETA_OFFSET
    print("w offset", start_xy)
    send_array(socket, start_xy)
    print(socket.recv_string())
    socket.send_string(A)
    print(socket.recv_string())
    socket.send_string(B)
    print(socket.recv_string())
    socket.send_string("Waiting for path")
    paths = recv_array(socket)
    print(paths)
    socket.send_string("Path received")
    #move_range = recv_array(socket)
    end_xyz = recv_array(socket)
    z = end_xyz[2]
    end_xyz = (n2r_matrix @ np.array([end_xyz[0], end_xyz[1], 1]))
    end_xyz[2] = z

    #if input("Start navigation? Y or N ") == 'N':
    #    return None
    
    # Let the robot run faster
    robot.nav.set_velocity(v = 25, w = 20)

    final_paths = []
    for path in paths:
        transformed_path = n2r_matrix @ np.array([path[0], path[1], 1])
        #path[0], path[1] = transformed_path[0], transformed_path[1]
        transformed_path[2] = path[2] - THETA_OFFSET
        #path = (path[0] - X_OFFSET, path[1] - Y_OFFSET, path[2] - THETA_OFFSET)
        print(transformed_path)
        final_paths.append(transformed_path)
        navigate(robot, transformed_path)
    xyt = robot.nav.get_base_pose()
    xyt[2] = xyt[2] + np.pi / 2
    navigate(robot, xyt)
    # last_waypoint = np.copy(final_paths[-1])
    # last_waypoint[2] += np.pi / 2
    # final_paths.append(last_waypoint)
    # print(last_waypoint)
    # robot.nav.execute_trajectory(
    #     final_paths, 
    #     pos_err_threshold = POS_TOL, 
    #     #rot_err_threshold = YAW_TOL, 
    #     per_waypoint_timeout = 30)
    #robot.wait_for_waypoints(xyt, pos_err_threshold = POS_TOL, rot_err_threshold = YAW_TOL, timeout = 50)
    return end_xyz

def run_manipulation(args, hello_robot, socket, text, transform_node, base_node, move_range = [False, False], top_down = False):
    
    gripper_pos = 1
    hello_robot.move_to_position(arm_pos=INIT_ARM_POS,
                                head_pan=INIT_HEAD_PAN,
                                head_tilt=INIT_HEAD_TILT,
                                gripper_pos = gripper_pos)
    time.sleep(1)
    hello_robot.move_to_position(lift_pos=INIT_LIFT_POS,
                                wrist_pitch = global_parameters.INIT_WRIST_PITCH,
                                wrist_roll = INIT_WRIST_ROLL,
                                wrist_yaw = INIT_WRIST_YAW)
    time.sleep(2)

    camera = RealSenseCamera(hello_robot.robot)

    args.mode = 'pick'
    args.picking_object = text
    rotation, translation, depth = capture_and_process_image(camera, args, socket, hello_robot, INIT_HEAD_TILT, top_down = top_down)
    
    #if input('Do you want to do this manipulation? Y or N ') != 'N':
    pickup(hello_robot, rotation, translation, base_node, transform_node, top_down = top_down, gripper_depth = depth)
    
    print("coordinates =", hello_robot.robot.nav.get_base_pose())

    print("Shift back to the original point")
    hello_robot.move_to_position(base_trans = -hello_robot.robot.manip.get_joint_positions()[0])
    print("coordinates =", hello_robot.robot.nav.get_base_pose())

def run_place(args, hello_robot, socket, text, transform_node, base_node, move_range = [False, False], top_down = False):

    camera = RealSenseCamera(hello_robot.robot)

    args.mode = 'place'
    args.placing_object = text
    time.sleep(2)
    print("Capture and process image")
    rotation, translation, _ = capture_and_process_image(camera, args, socket, hello_robot, INIT_HEAD_TILT, top_down = top_down)
    print(f"{rotation=}")
    hello_robot.move_to_position(lift_pos=1.1)
    time.sleep(1)
    hello_robot.move_to_position(wrist_yaw=0,
                                 wrist_pitch=0)
    time.sleep(1)
    time.sleep(1)
    move_to_point(hello_robot, translation, base_node, transform_node, move_mode=0)
    hello_robot.move_to_position(gripper_pos=1)
    hello_robot.move_to_position(lift_pos = hello_robot.robot.manip.get_joint_positions()[1] + 0.2)
    hello_robot.move_to_position(wrist_roll = 3)
    time.sleep(1)
    hello_robot.move_to_position(wrist_roll = -3)
    time.sleep(4)
    hello_robot.move_to_position(gripper_pos=1, 
                                lift_pos = 1.1,
                                arm_pos = 0)
    time.sleep(4)
    hello_robot.move_to_position(wrist_pitch=-1.57)
    time.sleep(1)
    hello_robot.move_to_position(base_trans = -hello_robot.robot.manip.get_joint_positions()[0])

def compute_tilt(camera_xyz, target_xyz):
    vector = camera_xyz - target_xyz
    return -np.arctan2(vector[2], np.linalg.norm(vector[:2]))

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
