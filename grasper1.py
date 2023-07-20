from robot import HelloRobot
from camera import DemoApp
from camera import CaptureImage
from global_parameters import *
import global_parameters
from args import get_args
from camera import RealSenseCamera, Visualizer
from segment import segment_image
from utils import potrait_to_landscape, segment_point_cloud, plane_detection, display_image_and_point

import time
import rospy
import cv2
import numpy as np
import sys
import PyKDL
from PIL import Image

if __name__ == "__main__":
    args = get_args()

    # Initalize robot and move to a height of 0.86
    if args.base_frame  == "gripper_camera":
        base_node = CAMERA_NODE
    elif args.base_frame == "top_camera":
        base_node = TOP_CAMERA_NODE
    elif args.base_frame == "gripper_left":
        base_node = GRIPPER_FINGERTIP_LEFT_NODE
    elif args.base_frame == "gripper_right":
        base_node = GRIPPER_FINGERTIP_RIGHT_NODE

    if args.transform_node == "gripper_left":
        transform_node = GRIPPER_FINGERTIP_LEFT_NODE
    elif args.transform_node == "gripper_right":
        transform_node = GRIPPER_FINGERTIP_RIGHT_NODE
    
    if (args.transform):
        hello_robot = HelloRobot(end_link=transform_node)
    else:
        hello_robot = HelloRobot(end_link=base_node)
    
    if args.mode == "move":
        gripper_pos = 0
    else:
        gripper_pos = 1
    
    # Moving robot to intital position
    hello_robot.move_to_position(lift_pos=INIT_LIFT_POS,
                                wrist_pitch = INIT_WRIST_PITCH,
                                gripper_pos = gripper_pos)
    time.sleep(10)

    # Intialsiing Camera
    camera = RealSenseCamera()

    # Image Capturing 
    rgb_image, depth_image, points = camera.capture_image()
    h, _, _ = rgb_image.shape

    # Rotating rgb, depth and point clouds by 90deg for segmnetation
    rotated_rgb = np.rot90(rgb_image, k=-1)
    rotated_depth = np.rot90(depth_image, k=-1)
    rotated_points = np.rot90(points, k=-1)

    # point selector
    if args.picking_object is None and args.placing_object is None:
        # Displaying windows for point selection
        ix, iy = camera.visualize_image()
        print(f"ix - {ix},iy - {iy}")

        # Image to world co-ordinates conversion
        sx, sy, sz = camera.pixel2d_to_point3d(ix, iy)
        point = PyKDL.Vector(sx, -sy, sz)
        print(f"x - {sx}, y - {sy}, z - {sz}")

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

    # Camera frame to gripper frame transformation
    if args.transform and transform_node is not None:
        transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
        transformed_point = transform * point
    else:
        transformed_point = point
    print(point, transformed_point)
    
    # Moving robot to a desired point
    hello_robot.move_to_pose(
        [transformed_point.x(), transformed_point.y(), transformed_point.z()],
        [0, 0, 0],  
        [gripper_pos]
    )
    
    # Picking the object
    if (args.mode == "pick"):
        hello_robot.pickup(abs(0))

    # final_rel_pos = PyKDL.Vector(0.5, 0, 0)
    # hello_robot.move_to_pose(final_rel_pos, [0,0,0], [0.5])