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
        cv2.imshow("RGB Image", rgb)
        cv2.imshow("Depth Image", depth)
        # cv2.waitKey(1000)
        if ix is not None and iy is not None:
            cv2.line(rgb, (ix-25, iy-25), (ix + 25, iy + 25), (0, 0, 255), 2)
            cv2.line(rgb, (ix-25, iy+25), (ix + 25, iy - 25), (0, 0, 255), 2)
            cv2.imshow("RGB Image", rgb)
            cv2.waitKey(3000)
            break
            # cv2.imwrite("./images/crossed_rgb.png", rgb)
            # break
        if cv2.waitKey(1000) == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Initalize robot and move to a height of 0.86
    # hello_robot = HelloRobot(end_link="link_gripper_fingertip_left")
    # hello_robot = HelloRobot()
    # hello_robot.move_to_position(lift_pos=0.86, gripper_pos = 0)

    # # Intialize Camera
    # app = DemoApp()
    # app.connect_to_device(dev_idx = 0)

    time.sleep(7)
    # # point = PyKDL.Vector(0, 0, -0.4)
    # # transform, frame2, frame1 = hello_robot.get_joint_transform('link_raised_gripper', 'link_gripper_fingertip_right')
    # # transformed_point1 = transform * point
    # # transformed_point = point
    # # transformed_point2 = ((frame1 * point))
    # # print(frame2, frame1, transform)
    # # print(point, transformed_point, transformed_point1)
    # # # transformed_point = point
    # # hello_robot.move_to_pose(
    # #         [transformed_point.x(), transformed_point.y(), transformed_point.z()],
    # #         [0, 0, 0],  
    # #         [0]
    # #     )
    # # process image
    # rgb, depth, intrinsic_mat = app.start_process_image()
    # resized_depth = upscale_depth(rgb, depth)
    # # print(np.asarray(resized_depth).shape[:2])
    # show_image(rgb, resized_depth)
    

    # # # pixel to pcd converison
    # print(ix, iy)
    # if (ix is not None) and (iy is not None):
    #     x, y, z = pixel_to_pcd(ix, iy, resized_depth, intrinsic_mat)
    #     print("depth, calc depth - ", np.asarray(resized_depth)[iy, ix], z)
    #     # transform for converting point in camera co-ordiantes to gripper finger tip co-ordiantes
    #     transform, frame2, frame1 = hello_robot.get_joint_transform('link_raised_gripper', 'link_gripper_fingertip_left')

    #     point = PyKDL.Vector(x, y, z)
    #     # transformed_point = transform * point
    #     transformed_point = point
    #     print(point, transformed_point)

    #     hello_robot.move_to_pose(
    #         [transformed_point.x(), transformed_point.y(), transformed_point.z()],
    #         [0, 0, 0],  
    #         [0]
    #     )
    #     # hello_robot.move_to_pose(
    #     #     [point.x(), point.y(), point.z()],
    #     #     [0, 0, 0],  
    #     #     [0]
    #     # )
    # # time.sleep(7)
    # # hello_robot.home()
    # # self.hello_robot.home()
