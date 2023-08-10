from robot import HelloRobot
from camera import DemoApp
from camera import CaptureImage
from global_parameters import *
import global_parameters
from args import get_args
from camera import RealSenseCamera, Visualizer
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
    
    if args.mode == "move":
        gripper_pos = 0
    else:
        gripper_pos = 1

    # Joint state publisher
    # pub_proc = Process(target = publisher_process, args=(hello_robot, ))
    # pub_proc.start()

    try:
        rospy.init_node('hello_robot_node')
    except:
        print('node already initialized hello_robot')

    # publisher = JointStatePublisher(hello_robot)
    # publisher.publish()

    # Frame transform listener
    # listener = Listener()
    # trans = listener.start("base_link", "camera_color_frame")
    # pub_proc.terminate()
    # exit()

    # Moving robot to intital position
    hello_robot.move_to_position(arm_pos=INIT_ARM_POS,
                                wrist_pitch = INIT_WRIST_PITCH,
                                wrist_roll = INIT_WRIST_ROLL,
                                wrist_yaw = INIT_WRIST_YAW,
                                gripper_pos = gripper_pos)
    time.sleep(2)

    hello_robot.move_to_position(lift_pos=INIT_LIFT_POS)

    time.sleep(2)

    # Intialsiing Camera
    # camera = RealSenseCamera()

    # image_publisher = ImagePublisher(camera)
    # image_publisher.publish_image()
    # exit()
    
    # hello_robot.move_to_pose(
    #     # [transformed_point.x(), transformed_point.y(), transformed_point.z()+0.03],
    #     [0, 0, 0],
    #     # [0, 1, 0],
    #     # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
    #     [0, 0, 3],
    #     # [transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[2]],
    #     [gripper_pos]
    # )
    # exit()

    # Image Capturing 
    # rgb_image, depth_image, points = camera.capture_image()
    # h, _, _ = rgb_image.shape

    # Rotating rgb, depth and point clouds by 90deg for segmnetation
    # rotated_rgb = np.rot90(rgb_image, k=-1)
    # rotated_depth = np.rot90(depth_image, k=-1)
    # rotated_points = np.rot90(points, k=-1)

    # point selector
    # if args.picking_object is None and args.placing_object is None:
    #     # Displaying windows for point selection
    #     ix, iy = camera.visualize_image()
    #     print(f"ix - {ix},iy - {iy}")

    #     # Image to world co-ordinates conversion
    #     sx, sy, sz = camera.pixel2d_to_point3d(ix, iy)
    #     point = PyKDL.Vector(sx, -sy, sz)
    #     print(f"x - {sx}, y - {sy}, z - {sz}")

    # pickup object 
    if args.picking_object is not None:     
        # Segmentation and bounding box calculation
        predictions = segment_image(args.picking_object)
        if len(predictions) == 0:
            print("No picking object found")
            sys.exit()

        # bounding box center - 3d co-ordinates 
        bbox_center = predictions['instances'].pred_boxes[0].get_centers()
        pix, piy = int(bbox_center[0][0].item()), int(bbox_center[0][1].item())
        ix, iy = potrait_to_landscape(pix, piy, h)
        print(f"pix - {pix}, piy - {piy}")
        print(f"ix - {ix},iy - {iy}")

        # Visualize segmented image with point
        seg_img = Image.open(f"./out_{args.picking_object}.png")
        seg_rgb_img = np.asanyarray(seg_img)
        display_image_and_point(seg_rgb_img, pix, piy)

        global_parameters.CORRECTION_Z -= 0.02

        # Image to world co-ordinates
        sx, sy, sz = camera.pixel2d_to_point3d(ix, iy)
        point = PyKDL.Vector(sx, -sy, sz)
        print(f"x - {sx}, y - {sy}, z - {sz}")

    #  placing object
    if args.placing_object  is not None:
        # point cloud of segmented object
        pcd = segment_point_cloud(rotated_rgb, rotated_depth, rotated_points, args.placing_object)
        pcd = pcd.voxel_down_sample(voxel_size=0.001)

        # Visualize segmented image with point
        seg_img = Image.open(f"./out_{args.placing_object}.png")
        seg_rgb_img = np.asanyarray(seg_img)
        display_image_and_point(seg_rgb_img)

        # object drop off plane detection
        [tx, ty, tz] = plane_detection(pcd, vis=True)
        point = PyKDL.Vector(tx, -ty, tz)
        global_parameters.CORRECTION_Y = 0.02

    # Temporary testing
    # tx = -0.02
    # ty = 0.14
    # tz = 0.73
    # point = PyKDL.Vector(-0.2854031, -0.01337335,  0.87199998)
    # point = PyKDL.Vector(-0.38634789, 0.01108794, 0.778 )
    # point = PyKDL.Vector(0.4, -0.5, 0)
    # point = PyKDL.Vector(0.10703942, -0.15927057, 0.66900003)
    point = PyKDL.Vector(0.10722475, 0.15421653,  0.67237347)
    # point[1] += 0.01
    # point[0] += 0.01
    global_parameters.CORRECTION_X, global_parameters.CORRECTION_Y, global_parameters.CORRECTION_Z = 0, 0, 0
    # rotation = PyKDL.Rotation(2.39388689e-01, -9.55172122e-01, -1.74181908e-01, 
    #                             -4.23444882e-02,  1.68956459e-01, -9.84713495e-01,
    #                             9.70000029e-01,  2.43104920e-01, -1.06264535e-08)
    
    # rotation = PyKDL.Rotation(-3.11710656e-01,  8.86093080e-01,  3.43038589e-01,
    #                             1.13836229e-01, -3.23599726e-01,  9.39321280e-01, 
    #                             9.43333328e-01,  3.31846684e-01, -1.45054795e-08)
    
    rotation = PyKDL.Rotation(0.07559296, -0.96571505, -0.24835478, 
                                 0.28120205, 0.25960431, -0.9238674,
                                  0.95666665, 0, 0.29118532)
    # rotation = PyKDL.Rotation(0.96571505,   -0.07559296, -0.24835478, 
    #                         -0.25960431,    -0.28120205, -0.9238674,
    #                         -0,             -0.95666665, 0.29118532)
    print(rotation.GetRPY())
    rotation = PyKDL.Rotation.RPY(rotation.GetRPY()[1], rotation.GetRPY()[0], rotation.GetRPY()[2]-1.53)
    print(rotation.GetRPY())
    dest_frame = PyKDL.Frame(rotation, point) 

    # Camera frame to gripper frame transformation
    if args.transform and transform_node is not None:
        transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        transformed_point = transform * point

        transformed_frame = transform * dest_frame

        # exit()
        if transform_node == GRIPPER_MID_NODE:
            transformed_point[2] -= 0.22
        # tf2 trans, rot
        # transform = listener.start(base_node, transform_node)
        # trans = transform.transform.translation
        # rot = transform.transform.rotation

        # # pyKDL trans, rot
        # trans_kdl = PyKDL.Vector(trans.x, trans.y, trans.z)
        # rot_kdl = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w)
        # frame_kdl = PyKDL.Frame(rot_kdl, trans_kdl)

        # transformed_point = frame_kdl * point
    else:
        transformed_point = point
    
    print(transformed_frame.p)
    print(transformed_frame.M.GetRPY())
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

    # hello_robot.move_to_pose(
    #     [transformed_point.x(), transformed_point.y(), transformed_point.z()],
    #     [0, 0, 0],
    #     # [-transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[2]+1.53],
    #     [gripper_pos]
    # )

    hello_robot.move_to_pose(
        # [transformed_point.x(), transformed_point.y(), transformed_point.z()+0.03],
        [0, 0, 0],
        # [0, 1, 0],
        # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        # [0, 0, 0],
        [transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[2]],
        [gripper_pos]
    )
    exit()

    hello_robot.move_to_pose(
        # [transformed_point.x(), transformed_point.y(), transformed_point.z()+0.03],
        [0, 0, 0],
        # [0, 1, 0],
        # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        [-transformed_frame.M.GetRPY()[1], -transformed_frame.M.GetRPY()[0], -transformed_frame.M.GetRPY()[2]],
        # [-transformed_frame.M.GetRPY()[1], -transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[2]+1.53],
        [gripper_pos]
    )

    time.sleep(5)

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