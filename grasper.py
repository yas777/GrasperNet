from robot import HelloRobot
from camera import DemoApp
from utils.robot_utils import pixel_to_pcd
from global_parameters import *
from args import get_args

import time
import rospy
import cv2
import numpy as np
import PyKDL

ix, iy = None, None
def click_event(event, x, y, flags, param):
    global ix, iy 
    if event == cv2.EVENT_LBUTTONDOWN:
        ix = x
        iy = y
        print("Selected point: ({}, {})".format(ix, iy))

def upscale_depth(rgb, depth_img):
    height, width = rgb.shape[:2]
    resized_depth_img = cv2.resize(depth_img, (width, height))

    return resized_depth_img

def show_image(rgb, depth):
    global ix, iy
    # Window for depth
    cv2.namedWindow("Depth Image")

    # Window for RGB Image
    cv2.namedWindow("RGB Image")
    cv2.setMouseCallback("RGB Image", click_event)

    width, height = rgb.shape[:2]
    print(width, height)
    cv2.resizeWindow("RGB Image", width, height)
    cv2.resizeWindow("Depth Image", width, height)
    while(1):
        rotated_rgb = cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE)
        rotated_depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("RGB Image", rotated_rgb)
        cv2.imshow("Depth Image", rotated_depth)
        # cv2.waitKey(1000)
        
        if ix is not None and iy is not None:
            cv2.line(rotated_rgb, (ix-25, iy-25), (ix + 25, iy + 25), (0, 0, 255), 2)
            cv2.line(rotated_rgb, (ix-25, iy+25), (ix + 25, iy - 25), (0, 0, 255), 2)
            tiy = iy
            iy = width - ix
            ix = tiy
            cv2.imshow("RGB Image", rotated_rgb)
            cv2.waitKey(3000)
            break
            # cv2.imwrite("./images/crossed_rgb.png", rgb)
            # break
        if cv2.waitKey(1000) == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    args = get_args()

    # Initalize robot and move to a height of 0.86
    if args.base_frame  == "gc":
        base_node = CAMERA_NODE
    elif args.base_frame == "tc":
        base_node = TOP_CAMERA_NODE

    if args.transform_node == "gl":
        transform_node = GRIPPER_FINGERTIP_LEFT_NODE
    elif args.transform_node == "gr":
        transform_node = GRIPPER_FINGERTIP_RIGHT_NODE
    
    if (args.transform):
        hello_robot = HelloRobot(end_link=transform_node)
    else:
        hello_robot = HelloRobot(end_link=base_node)
    
    if args.mode == "m":
        gripper_pos = 0
    else:
        gripper_pos = 1
    
    hello_robot.move_to_position(lift_pos=0.86, gripper_pos = gripper_pos)

    # # Intialize Camera
    app = DemoApp()
    app.connect_to_device(dev_idx = 0)

    # ALlowing the robot to settle before taking the picture 
    time.sleep(7)

    # # process image
    rgb, depth, intrinsic_mat = app.start_process_image()
    resized_depth = upscale_depth(rgb, depth)
    # print(np.asarray(resized_depth).shape[:2])
    show_image(rgb, resized_depth)
    

    # pixel to pcd converison
    print(ix, iy, args.transform)
    # ix, iy = 1, 2
    if (ix is not None) and (iy is not None):
        x, y, z = pixel_to_pcd(ix, iy, resized_depth, intrinsic_mat)
        point = PyKDL.Vector(x, y, z)
        print(z)
        # point = PyKDL.Vector(0, 0, 0.3)
        
        # transform for converting point in camera co-ordiantes to gripper finger tip co-ordiantes
        if args.transform and transform_node is not None:
            transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
            transformed_point = transform * point
        else:
            transformed_point = point
        
        print(point, transformed_point)

        hello_robot.move_to_pose(
            [transformed_point.x(), transformed_point.y(), transformed_point.z()],
            [0, 0, 0],  
            [gripper_pos]
        )
        
        if (args.mode == "pi"):
            hello_robot.pickup(abs(z))