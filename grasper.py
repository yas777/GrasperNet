from robot import HelloRobot
from camera import DemoApp
from camera import CaptureImage
from utils.robot_utils import pixel_to_pcd
from utils.seg_utils import visualize_masks
from global_parameters import *
from args import get_args
from camera.segmentation import Segment
from camera import RealSenseCamera

import time
import rospy
import cv2
import numpy as np
import sys
import PyKDL

np.set_printoptions(threshold=sys.maxsize)
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
    cv2.setMouseCallback("Depth Image", click_event)

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
        # print(rotated_depth.dtype)
        # rotated_depth_colored = None
        # cv2.applyColorMap(rotated_depth, rotated_depth_colored, cv2.COLORMAP_BONE)
        cv2.imshow("RGB Image", rotated_rgb)
        cv2.imshow("Depth Image", rotated_depth/np.max(rotated_depth))
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
    
    hello_robot.move_to_position(lift_pos=INIT_LIFT_POS, 
                                gripper_pos = gripper_pos,
                                wrist_pitch=INIT_WRIST_PITCH)

    # # Intialize Camera
    # if args.mode == "pick":
    # if base_node == TOP_CAMERA_NODE:
    #     app = CaptureImage()
    # elif base_node == CAMERA_NODE:
    #     app = DemoApp()
    #     app.connect_to_device(dev_idx = 0)

    # ALlowing the robot to settle before taking the picture 
    time.sleep(7)

    # # process image
    # rgb, depth, intrinsic_mat = app.start_process_image()
    # resized_depth = upscale_depth(rgb, depth)
    # np.save("depth.npy", resized_depth)
    # print(np.asarray(resized_depth).shape[:2])

    # sam = Segment(SAM_CHECKPOINT, SAM_MODEL_TYPE)
    # sam.set_image(rgb, resized_depth)
    # show_image(rgb, resized_depth)

    # pixel to pcd converison
    # print(ix, iy, args.transform)
    camera = RealSenseCamera()
    camera.capture_image()
    ix, iy = camera.visualize_image()
    print(ix, iy)
    if (ix is not None) and (iy is not None):
        # input_points = np.array([[ix ,iy]])
        # input_labels = np.array([1])
        # masks, scores, min_y, max_y = sam.segment_image(input_points, input_labels)
        # visualize_masks(rgb, masks, scores, input_points, input_labels)
        # print(min_dep, max_dep)
        # exit()

        x, y, z = pixel_to_pcd(ix, iy, resized_depth, intrinsic_mat)
        # new_z = (min_dep + max_dep) / 2
        point = PyKDL.Vector(x, -y, z)
        print(x, y, z)

        # ix1, iy1, ix2, iy2 = ix, max_y, ix, min_y
        # x1, y1, z1 = pixel_to_pcd(ix1, iy1, resized_depth, intrinsic_mat)
        # x2, y2, z2 = pixel_to_pcd(ix2, iy2, resized_depth, intrinsic_mat)
        # print(x1, y1, z1)
        # print(x2, y2, z2)
        # exit()
        # point = PyKDL.Vector(0, 0, 0.3)
        
        # transform for converting point in camera co-ordiantes to gripper finger tip co-ordiantes
        if args.transform and transform_node is not None:
            transform, frame2, frame1 = hello_robot.get_joint_transform(base_node, transform_node)
            transformed_point = transform * point
            print("frame1 - ", frame1)
            print("frame2 - ", frame2)
            print("transform - ", transform)
        else:
            transformed_point = point
        
        print(point, transformed_point)
        
        hello_robot.move_to_pose(
            [transformed_point.x(), transformed_point.y(), transformed_point.z()],
            [0, 0, 0],  
            [gripper_pos]
        )
        
        if (args.mode == "pick"):
            hello_robot.pickup(abs(0))