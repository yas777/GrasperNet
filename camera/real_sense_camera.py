#!/usr/bin/env python3

import rospy
import sys
import os
import cv2
import time
import numpy as np

from utils.robot_utils import cam_info_to_intrinsic_mat
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class CaptureImage:
    """
    A class that converts a subscribed ROS image to a OpenCV image and saves
    the captured image to a predefined directory.
    """
    def __init__(self):
        """
        A function that initializes a CvBridge class, subscriber, and save path.
        :param self: The self reference.
        """
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.intrinsic_mat = None
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback_rgb, queue_size=1)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.call_back_depth, queue_size=1)
        self.cam_info = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.callback_info, queue_size=1)
        self.save_path = '/home/hello-robot-yaswanth/camera_test'

    def callback_rgb(self, msg):
        """
        A callback function that converts the ROS image to a CV2 image and stores the
        image.
        :param self: The self reference.
        :param msg: The ROS image message type
        .
        """
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge error: {0}'.format(e))
        
        print("calling callback rgb", self.rgb.shape)
        # print("Image", self.rgb)
        # time.sleep(10)
        # cv2.namedWindow("Image")
        # cv2.imshow("Image", image)
        # cv2.waitKey(10000)
        # file_name = 'camera_image.jpeg'
        # completeName = os.path.join(self.save_path, file_name)
        # cv2.imwrite(completeName, image)
        rospy.signal_shutdown("done")
        sys.exit(0)
    
    def call_back_depth(self, msg):
        """
        A callback function that converts the ROS image to a CV2 image and stores the
        image.
        :param self: The self reference.
        :param msg: The ROS image message type.
        """

        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge error: {0}'.format(e))
        
        self.depth = np.array(self.depth, dtype=np.float32)
        print("calling depth callback", self.depth.shape)
        # print("depth", self.depth)
        # time.sleep(100)
        # cv2.namedWindow("Depth")
        # cv2.imshow("Depth", depth)
        # cv2.waitKey(10000)
        # file_name = 'camera_image.jpeg'
        # completeName = os.path.join(self.save_path, file_name)
        # cv2.imwrite(completeName, image)
        rospy.signal_shutdown("done")
        sys.exit(0)

    def callback_info(self, msg):
        print(msg)
        self.intrinsic_mat = cam_info_to_intrinsic_mat(msg.K)
        print("info", self.intrinsic_mat)
        rospy.signal_shutdown("done")
        sys.exit(0)

    def start_process_image(self):
        print("calling process iamge")
        time.sleep(5)
        while True:
            # print(self.rgb, self.depth, self.intrinsic_mat)
            if (self.rgb is not None) and (self.depth is not None) and (self.intrinsic_mat is not None):
                return self.rgb, self.depth, self.intrinsic_mat


if __name__ == '__main__':
    rospy.init_node('capture_image', argv=sys.argv)
    CaptureImage()
    rospy.spin()

