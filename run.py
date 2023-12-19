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

from utils.grasper_utils import pickup, move_to_point
from grasper2 import capture_and_process_image


#robot = StretchClient()
#robot.switch_to_navigation_mode()
#robot.move_to_nav_posture()
#robot = stretch_body.robot.Robot()
#robot.startup()

POS_TOL = 0.1
YAW_TOL = 0.2

X_OFFSET = 0.241888
Y_OFFSET = -0.160611
# x1 = X_OFFSET, x2 = another x
# THETA_OFFSET =  np.arctan2((x2 - x1), (y2 - y1))
THETA_OFFSET = 0.5565652208883989

r2n_matrix = \
    np.array([
        [1, 0, X_OFFSET],
        [0, 1, Y_OFFSET],
        [0, 0, 1]
    ]) @ \
    np.array([
        [np.cos(THETA_OFFSET), -np.sin(THETA_OFFSET), 0],
        [np.sin(THETA_OFFSET), np.cos(THETA_OFFSET), 0],
        [0, 0, 1]
    ])

n2r_matrix = \
    np.array([
        [np.cos(THETA_OFFSET), np.sin(THETA_OFFSET), 0],
        [-np.sin(THETA_OFFSET), np.cos(THETA_OFFSET), 0],
        [0, 0, 1]
    ]) @ \
    np.array([
        [1, 0, -X_OFFSET],
        [0, 1, -Y_OFFSET],
        [0, 0, 1]
    ])

def navigate(robot, xyt_goal):
    xyt_goal = np.asarray(xyt_goal)
    while xyt_goal[2] < -np.pi or xyt_goal[2] > np.pi:
        xyt_goal[2] = xyt_goal[2] + 2 * np.pi if xyt_goal[2] < -np.pi else xyt_goal[2] - 2 * np.pi
    while True:
        robot.nav.navigate_to(xyt_goal, blocking = False)
        xyt_curr = robot.nav.get_base_pose()
        print("The robot currently loactes at " + str(xyt_curr))
        #time.sleep(0.5)
        if np.allclose(xyt_curr[:2], xyt_goal[:2], atol=POS_TOL) and \
                (np.allclose(xyt_curr[2], xyt_goal[2], atol=YAW_TOL)\
                 or np.allclose(xyt_curr[2], xyt_goal[2] + np.pi * 2, atol=YAW_TOL)\
                 or np.allclose(xyt_curr[2], xyt_goal[2] - np.pi * 2, atol=YAW_TOL)):
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

def read_input():
    A = str(input("Enter A: "))
    print("A = ", A)
    B = str(input("Enter B: "))
    print("B = ", B)

    return A, B

def run_navigation(robot, socket, A, B):
    # Reset robot
    #print("Resetting robot...")
    #robot.reset()
    start_xy = robot.nav.get_base_pose()
    print(start_xy)
    #start_xy[0] += X_OFFSET
    #start_xy[1] += Y_OFFSET
    transformed_start_xy = r2n_matrix @ np.array([start_xy[0], start_xy[1], 1])
    start_xy[0], start_xy[1] = transformed_start_xy[0], transformed_start_xy[1]
    start_xy[2] += THETA_OFFSET
    print(start_xy)
    # A = str(input("Enter A: "))
    # print("A = ", A)
    # B = str(input("Enter B: "))
    # print("B = ", B)
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

    if input("Start navigation? Y or N ") == 'N':
        return None
    
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
    time.sleep(2)

    camera = RealSenseCamera(hello_robot.robot)

    args.mode = 'pick'
    args.picking_object = text
    rotation, translation, depth = capture_and_process_image(camera, args, socket, hello_robot, INIT_HEAD_TILT, top_down = top_down)

    if input('Do you want to do this manipulation? Y or N ') != 'N':
        pickup(hello_robot, rotation, translation, base_node, transform_node, top_down = top_down, gripper_depth = depth)

    # Shift back to the original point
    hello_robot.move_to_position(base_trans = -hello_robot.robot.manip.get_joint_positions()[0])

    # gripper_pos = 1
    # global_parameters.INIT_WRIST_PITCH = -1.57
    
    # hello_robot.move_to_position(arm_pos=INIT_ARM_POS,
    #                             head_pan=INIT_HEAD_PAN,
    #                             head_tilt=INIT_HEAD_TILT,
    #                             gripper_pos = gripper_pos)
    
    # hello_robot.move_to_position(lift_pos=INIT_LIFT_POS,
    #                             wrist_pitch = global_parameters.INIT_WRIST_PITCH,
    #                             wrist_roll = INIT_WRIST_ROLL,
    #                             wrist_yaw = INIT_WRIST_YAW)

    # head_tilt_angles = [0.15, 0.05, -0.05, -0.15]
    # tilt_retries, side_retries = 0, 0
    # retry_flag = True
    # head_tilt = INIT_HEAD_TILT
    # head_pan = INIT_HEAD_PAN

    # while(retry_flag):
    #     print(f"side, tilt retries {side_retries}, {tilt_retries}")
    #     translation, rotation, depth, cropped, retry_flag = image_publisher.publish_image(text, head_tilt=head_tilt)

    #     if (retry_flag == 1):
    #         base_trans = translation[0]
    #         if base_trans < 0:
    #             base_trans = max(-0.1 if move_range[1] else 0, base_trans)
    #         else:
    #             base_trans = min(0.1 if move_range[0] else 0, base_trans)
    #         head_tilt += (rotation[0])

    #         hello_robot.move_to_position(base_trans=base_trans,
    #                                 head_pan=head_pan,
    #                                 head_tilt=head_tilt)

    #     elif retry_flag == 2:
    #         if (side_retries == 2 and tilt_retries == 4):
    #             hello_robot.move_to_position(base_trans=0.15, head_tilt=head_tilt)
    #             side_retries = 3
    #         elif (tilt_retries == 4):
    #             if (side_retries == 0):
    #                 hello_robot.move_to_position(base_trans= -0.1 if move_range[1] else 0, head_tilt=head_tilt)
    #                 side_retries = 1
    #             else:
    #                 hello_robot.move_to_position(base_trans= 0.2 if move_range[0] else 0, head_tilt=head_tilt)
    #                 side_retries = 2
    #             tilt_retries = 0
    #         elif side_retries == 3:
    #             print("No poses found in all retries")
    #             exit()
    #         else:
    #             print(f"retrying with head tilt : {head_tilt + head_tilt_angles[tilt_retries]}")
    #             hello_robot.move_to_position(head_pan=head_pan,
    #                                     head_tilt=head_tilt + head_tilt_angles[tilt_retries])
    #             tilt_retries += 1
                
    #     elif (retry_flag == 0):
    #         retry_flag = False

    #     time.sleep(1)

    # point = PyKDL.Vector(-translation[1], -translation[0], translation[2])
        
    # rotation1 = PyKDL.Rotation(rotation[0][0], rotation[0][1], rotation[0][2],
    #                             rotation[1][0],  rotation[1][1], rotation[1][2],
    #                             rotation[2][0],  rotation[2][1], rotation[2][2])

    # # Rotation from camera frame to model frame
    # rotation1_bottom = PyKDL.Rotation(0.0000000, -1.0000000,  0.0000000,
    #                                 -1.0000000,  0.0000000,  0.0000000, 
    #                                 0.0000000,  0.0000000, 1.0000000)
    # print(f"Points frame rotation - {rotation1.GetRPY()}, {rotation1.GetEulerZYX()}")

    # # Rotation from camera frame to pose frame
    # rotation =  rotation1_bottom * rotation1
    # print(f"Camera frame rotation - {rotation.GetRPY()}, {rotation.GetEulerZYX()}")

    # dest_frame = PyKDL.Frame(rotation, point) 

    # hello_robot.move_to_position(lift_pos = 1.1, head_pan = None, head_tilt = None)
    # # transform - Rotation and translation from camera frame to gripper frame
    # transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
    # transformed_frame = transform * dest_frame

    # print("pose: ", transformed_frame.p)
    # print("rotation: ", transformed_frame.M.GetRPY())

    # # Rotation for aligning gripper frame to model pose frame
    # rotation2_top = PyKDL.Rotation(0, 0, 1, 1, 0, 0, 0, -1, 0)
        
    # # final Rotation of gripper to hold the objet
    # final_rotation = transformed_frame.M * rotation2_top

    # # Only rotating the gripper 
    # hello_robot.move_to_pose(
    #     [0, 0, 0],
    #     [final_rotation.GetRPY()[0], final_rotation.GetRPY()[1], final_rotation.GetRPY()[2]],
    #     [gripper_pos],
    # )
    # time.sleep(1)

    # # Calculating new co-rodinates of pose center
    # if args.transform and transform_node is not None:
    #         transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
    #         transformed_point1 = transform * point
            
    #         print(f"transformed point1 : {transformed_point1}")
    #         transformed_point1[2] -= (0.185)
    #         ref_diff = 0.185
        
    # # Moving gripper to pose center
    # hello_robot.move_to_pose(
    #     [transformed_point1.x(), transformed_point1.y(), transformed_point1.z() - 0.2],
    #     [0, 0, 0],
    #     [gripper_pos],
    #     move_mode = 1
    # )
    # time.sleep(1)
    # transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
    # transformed_point2 = transform * point
    # print(f"transformed point2 : {transform * point}")
    # curr_diff = transformed_point2.z()

    # diff = abs(curr_diff - ref_diff)
    # velocities = [1]*8
    # velocities[5:] = [0.007, 0.007, 0.007, 0.007]
    # velocities[0] = 0.005
    # if diff > 0.08:
    #     dist = diff - 0.08
    #     hello_robot.move_to_pose(
    #         [0, 0, dist],
    #         [0, 0, 0],
    #         [gripper_pos]
    #     )
    #     time.sleep(1)
    #     transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
    #     print(f"transformed point3 : {transform * point}")
    #     diff = diff - dist
            
    # while diff > 0.01:
    #     dist = min(0.03, diff)
    #     hello_robot.move_to_pose(
    #         [0, 0, dist],   
    #         [0, 0, 0],
    #         [gripper_pos],
    #         velocities=velocities
    #     )
    #     time.sleep(1)
    #     transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
    #     print(f"transformed point3 : {transform * point}")
    #     diff = diff - dist
    # hello_robot.pickup(abs(0))
    # # Put it down for now
    # time.sleep(5)
    # hello_robot.move_to_position(gripper_pos = 1)
    # hello_robot.move_to_position(arm_pos = INIT_ARM_POS)
    # hello_robot.move_to_position(lift_pos = 1.1)
    # hello_robot.move_to_position(wrist_pitch = global_parameters.INIT_WRIST_PITCH)

def run_place(args, hello_robot, socket, text, transform_node, base_node, move_range = [False, False], top_down = False):
    
    # gripper_pos = 1

    # print(INIT_ARM_POS, INIT_WRIST_PITCH, INIT_WRIST_ROLL, INIT_WRIST_YAW, gripper_pos)
    # hello_robot.move_to_position(arm_pos=INIT_ARM_POS,
    #                             head_pan=INIT_HEAD_PAN,
    #                             head_tilt=INIT_HEAD_TILT,
    #                             gripper_pos = gripper_pos)
    # time.sleep(1)
    
    # hello_robot.move_to_position(lift_pos=INIT_LIFT_POS,
    #                             wrist_pitch = global_parameters.INIT_WRIST_PITCH,
    #                             wrist_roll = INIT_WRIST_ROLL,
    #                             wrist_yaw = INIT_WRIST_YAW)
    # time.sleep(2)

    camera = RealSenseCamera(hello_robot.robot)

    args.mode = 'place'
    args.placing_object = text
    rotation, translation, _ = capture_and_process_image(camera, args, socket, hello_robot, INIT_HEAD_TILT, top_down = top_down)

    hello_robot.move_to_position(lift_pos=1.1)
    time.sleep(1)
    hello_robot.move_to_position(wrist_yaw=0,
                                 wrist_pitch=0)
    time.sleep(1)
    # hello_robot.move_to_position(wrist_yaw=0)
    #hello_robot.move_to_position(lift_pos=1.1)
    # hello_robot.move_to_position(wrist_pitch=0)
    time.sleep(1)
    move_to_point(hello_robot, translation, base_node, transform_node)
    time.sleep(4)
    hello_robot.move_to_position(gripper_pos=1, 
                                lift_pos = 1.1,
                                arm_pos = 0)
    time.sleep(4)
    #hello_robot.move_to_position(wrist_pitch=-1.57, arm_pos = 0)
    #if abs(robot.robot.manip.get_joint_positions()[3] - 2.5) > 0.1:
    #    hello_robot.move_to_position(wrist_yaw  = - 2.5)
    # hello_robot.move_to_position(lift_pos=1.1)
    # hello_robot.move_to_position(arm_pos=0)
    hello_robot.move_to_position(wrist_pitch=-1.57)

def compute_tilt(camera_xyz, target_xyz):
    vector = camera_xyz - target_xyz
    return -np.arctan2(vector[2], np.linalg.norm(vector[:2]))

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
    #topdown_socket.connect("tcp://" + "100.107.224.62" + ":" + str(args.manipulation_port + 2))
    #manip_socket.connect("tcp://172.24.71.253:5556")

    while True:
        A = None
        if input("You want to run navigation? Y or N") != "N":
            A, B = read_input()

            hello_robot.robot.switch_to_navigation_mode()
            hello_robot.robot.move_to_post_nav_posture()
            hello_robot.robot.head.look_front()
            end_xyz = run_navigation(hello_robot.robot, nav_socket, A, B)
            if not end_xyz is None:
                camera_xyz = hello_robot.robot.head.get_pose()[:3, 3]
                INIT_HEAD_TILT = compute_tilt(camera_xyz, end_xyz)

        #xyt = hello_robot.robot.nav.get_base_pose()
        #xyt[2] = xyt[2] + np.pi / 2
        #print('\n\n\n\n\n debug')
        #time.sleep(5)
        #navigate(hello_robot.robot, xyt)
        if input("You want to run manipulation? Y or N ") != 'N':
            if (A is None):
                A, _ = read_input()
        
            hello_robot.robot.switch_to_manipulation_mode()
            hello_robot.robot.head.look_at_ee()
            run_manipulation(args, hello_robot, anygrasp_socket, A, transform_node, base_node)
            #run_manipulation(args, hello_robot, anygrasp_open_socket, A, transform_node, base_node, move_range)
            #run_manipulation(args, hello_robot, topdown_socket, A, transform_node, base_node, move_range, top_down = True)
        
        if input("You want to run navigation? Y or N") != "N":
            A, B = read_input()

            hello_robot.robot.switch_to_navigation_mode()
            # hello_robot.robot.move_to_post_nav_posture()
            hello_robot.robot.head.look_front()
            end_xyz = run_navigation(hello_robot.robot, nav_socket, A, B)
            if not end_xyz is None:
                camera_xyz = hello_robot.robot.head.get_pose()[:3, 3]
                INIT_HEAD_TILT = compute_tilt(camera_xyz, end_xyz)

        #xyt = hello_robot.robot.nav.get_base_pose()
        #xyt[2] = xyt[2] + np.pi / 2
        #navigate(hello_robot.robot, xyt)
        if input("You want to run place? Y or N") != 'N':
            if (A is None):
                A, _ = read_input()
            hello_robot.robot.switch_to_manipulation_mode()
            # hello_robot.robot.move_to_manip_posture()
            hello_robot.robot.head.look_at_ee()
            run_place(args, hello_robot, anygrasp_socket, A, transform_node, base_node)
        time.sleep(1)

if __name__ == '__main__':
    run()
