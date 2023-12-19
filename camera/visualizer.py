import cv2
import numpy as np

class Visualizer():
    def __init__(self, rgb_image, depth_image, intrinsic_mat):
        self.rgb_image = rgb_image
        self.depth_image = depth_image

        _, w = self.depth_image.shape
        
        self.fx = intrinsic_mat[0][0]
        self.fy = intrinsic_mat[1][1]
        self.cx = intrinsic_mat[0][2]
        self.cy = intrinsic_mat[1][2]
        print(self.fx, self.fy, self.cx, self.cy)

        # selected point co-ordinates 
        self.ix, self.iy = None, None

    def pixel2d_to_point3d(self, ix, iy):
        d = self.depth_image[iy, ix]
        print(d)
        z = d/1000
        x = (ix - self.cx)*(abs(z))/self.fx
        y = -(iy - self.cy)*(abs(z))/self.fy

        return x, y, z

    def visualize_point_selector(self):

        # Window for Depth Image
        cv2.namedWindow("Depth Image")
        cv2.setMouseCallback("Depth Image", self.click_event)

        # Window for RGB Image
        cv2.namedWindow("RGB Image")
        cv2.setMouseCallback("RGB Image", self.click_event)

        width, height = self.rgb_image.shape[:2]
        cv2.resizeWindow("RGB Image", width, height)
        cv2.resizeWindow("Depth Image", width, height)

        while(1):
            rotated_rgb = cv2.rotate(self.rgb_image, cv2.ROTATE_90_CLOCKWISE)
            rotated_depth = cv2.rotate(self.depth_image, cv2.ROTATE_90_CLOCKWISE)

            cv2.imshow("RGB Image", rotated_rgb)
            cv2.imshow("Depth Image", rotated_depth/np.max(rotated_depth))
            # cv2.waitKey(1000)
            
            if self.ix is not None and self.iy is not None:
                cv2.line(rotated_rgb, (self.ix-25, self.iy-25), (self.ix + 25, self.iy + 25), (0, 0, 255), 2)
                cv2.line(rotated_rgb, (self.ix-25, self.iy+25), (self.ix + 25, self.iy - 25), (0, 0, 255), 2)
                tiy = self.iy
                self.iy = width - self.ix
                self.ix = tiy
                cv2.imshow("RGB Image", rotated_rgb)
                cv2.waitKey(3000)
                break
                # cv2.imwrite("./images/crossed_rgb.png", rgb)
                # break
            if cv2.waitKey(1000) == 27:
                break
            
        return self.ix, self.iy 

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.ix = x
            self.iy = y
            print("Selected point: ({}, {})".format(self.ix, self.iy))
