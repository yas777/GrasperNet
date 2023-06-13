import os 
print(os.getcwd())

from robot import HelloRobot
from camera import DemoApp
from utils.robot_utils import pixel_to_pcd
import time
import rospy
import cv2
import numpy as np
import PyKDL
import argparse

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
        rotateted_rgb = cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("RGB Image", rotateted_rgb)
        cv2.imshow("Depth Image", depth)
        # cv2.waitKey(1000)
        
        if ix is not None and iy is not None:
            cv2.line(rotateted_rgb, (ix-25, iy-25), (ix + 25, iy + 25), (0, 0, 255), 2)
            cv2.line(rotateted_rgb, (ix-25, iy+25), (ix + 25, iy - 25), (0, 0, 255), 2)
            tiy = iy
            iy = width - ix
            ix = tiy
            cv2.imshow("RGB Image", rotateted_rgb)
            cv2.waitKey(3000)
            break
            # cv2.imwrite("./images/crossed_rgb.png", rgb)
            # break
        if cv2.waitKey(1000) == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-bf", "--base_frame",
                    choices = ["c", "gl", "gr"], default = "c",
                    help = "Operating frame of reference")
    parser.add_argument("-t", "--transform", 
                    action="store_true", 
                    help = "Boolean for transforming a input co-ordinates to another frame of reference")
    parser.add_argument("-tn", "--transform_node", 
                    choices = ["c", "gl", "gr"], default = "gl",
                    help = "Operating frame of reference")
    args = parser.parse_args()

    

    # Initalize robot and move to a height of 0.86
    if args.base_frame  == "c":
        base_node = "link_raised_gripper"
    elif args.base_frame == "gl":
        base_node = "link_gripper_fingertip_left"
    elif args.base_frame == "gr":
        base_node = "link_gripper_fingertip_right"

    if args.transform_node == "c":
        transform_node = "link_raised_gripper"
    elif args.transform_node == "gl":
        transform_node = "link_gripper_fingertip_left"
    elif args.transform_node == "gr":
        transform_node = "link_gripper_fingertip_right"
    
    hello_robot = HelloRobot(end_link=transform_node)
    hello_robot.move_to_position(lift_pos=0.86, gripper_pos = 0)

    # # Intialize Camera
    app = DemoApp()
    app.connect_to_device(dev_idx = 0)

    # ALlowing the robot to settle before taking the picture 
    time.sleep(7)

    # # process image
    rgb, depth, intrinsic_mat = app.start_process_image()
    resized_depth = upscale_depth(rgb, depth)
    print(np.asarray(resized_depth).shape[:2])
    show_image(rgb, resized_depth)
    

    # pixel to pcd converison
    print(ix, iy, args.transform)
    if (ix is not None) and (iy is not None):
        x, y, z = pixel_to_pcd(ix, iy, resized_depth, intrinsic_mat)
        point = PyKDL.Vector(x, y, z)
        # point = PyKDL.Vector(0, 0, -0.3)
        
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
            [0]
        )

