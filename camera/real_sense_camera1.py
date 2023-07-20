## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

import open3d as o3d

import matplotlib.pyplot as plt

class RealSenseCamera:
    def __init__(self):
        
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        # Camera intrinsics
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx = intrinsics.fx
        self.fy = intrinsics.fy
        self.cx = intrinsics.ppx
        self.cy = intrinsics.ppy
        print(self.fx, self.fy, self.cx, self.cy)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 3 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.pc = rs.pointcloud()

        # selected ix and iy co-ordinates
        self.ix, self.iy = None, None

    def capture_image(self):

        # Streaming loop
        aligned_depth_frame, color_frame = None, None
        try:
            # while not aligned_depth_frame or not color_frame:
            # while True:
            for i in range(40):
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()
                # frames.get_depth_frame() is a 640x360 depth image

                # Align the depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                # if not aligned_depth_frame or not color_frame:
                #     continue

            self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
            # print(depth_image.max())
            self.rgb_image = np.asanyarray(color_frame.get_data())
            self.points = np.asanyarray(self.pc.calculate(aligned_depth_frame).get_vertices()).view(np.float32).reshape(240, 424 ,3)

            print(self.points.shape)

            # rotating the img by 90 deg
            # self.depth_image = np.rot90(self.depth_image, k=-1)
            # self.rgb_image = np.rot90(self.rgb_image, k=-1)
            # self.points = np.rot90(self.points, k=-1)

            cv2.imwrite("input.jpg", np.rot90(self.rgb_image, k=-1))
            cv2.imwrite("depth.jpg", np.rot90(self.depth_image, k=-1)/np.max(self.depth_image))
            self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
            np.save("rgb.npy", self.rgb_image)
            np.save("depth.npy", self.depth_image)
            np.save("points.npy", self.points)
            # o3d.io.write_point_cloud("pc.pcd", self.points)
            # Remove background - Set pixels further than clipping_distance to grey
            # grey_color = 153
            # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            # self.rgb_image = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            # bg_removed = color_image

            # Render images:
            #   depth align to color on left
            #   depth on right

            fig, ax = plt.subplots(1, 2, figsize=(10,5))
            timer = fig.canvas.new_timer(interval = 5000) #creating a timer object and setting an interval of 3000 milliseconds
            timer.add_callback(lambda : plt.close())

            ax[0].imshow(np.rot90(self.rgb_image, k=-1))
            ax[0].set_title("Color Image")

            ax[1].imshow(np.rot90(self.depth_image, k=-1))
            ax[1].set_title("Depth Image")
            
            plt.savefig("rgb_dpt.png")
            plt.pause(3)
            plt.close()
            # timer.start()
            # plt.show()
            
            # self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # # print(depth_colormap.shape, depth_colormap.max())
            # images = np.hstack((self.rgb_image, self.depth_colormap))
            

            # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
            # cv2.imshow('Align Example', images)
            # key = cv2.waitKey(1000)
            # # Press esc or 'q' to close the image window
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     # break   
            return self.rgb_image, self.depth_image, self.points
            # fig, axs = plt.subplots(1,2, figsize=(10, 5))

            # axs[0].imshow(bg_removed)
            # axs[0].axis('off')
            # axs[0].set_title("Color Image")

            # color_map = plt.cm.jet
            # cbar = fig.colorbar(plt.cm.ScalarMappable(cmap=color_map), ax=axs[0], shrink=0.6)
            # pcm = plt.pcolormesh(bg_removed, cmap="gray")
            # cbar = fig.colorbar(pcm, ax=axs[0], shrink=0.6)
            
            # axs[1].imshow(depth_colormap)
            # axs[1].axis('off')
            # axs[1].set_title('Depth Image')
            # axs[1].set_aspect('auto')

            # color_map1 = plt.cm.jet
            # cbar = fig.colorbar(plt.cm.ScalarMappable(cmap=color_map1), ax=axs[1], fraction=0.046, pad=0.04)
            # print(depth_colormap.shape)
            # print(depth_image.shape)
            # pcm = plt.pcolormesh(depth_image, cmap="Greys")
            # cbar = fig.colorbar(pcm, ax=axs[1], fraction=0.046, pad=0.04)

            # fig.tight_layout()
            
            # plt.show()
        finally:
            self.pipeline.stop()

    def pixel2d_to_point3d(self, ix, iy):
        d = self.depth_image[iy, ix]
        print(d)
        z = d*self.depth_scale
        x = (ix - self.cx)*(abs(z))/self.fx
        y = -(iy - self.cy)*(abs(z))/self.fy

        return x, y, z

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.ix = x
            self.iy = y
            print("Selected point: ({}, {})".format(self.ix, self.iy))

    def visualize_image(self):

        # Window for Depth Image
        cv2.namedWindow("Depth Image")
        cv2.setMouseCallback("Depth Image", self.click_event)

        # Window for RGB Image
        cv2.namedWindow("RGB Image")
        cv2.setMouseCallback("RGB Image", self.click_event)

        width, height = self.rgb_image.shape[:2]
        cv2.resizeWindow("RGB Image", width, height)
        cv2.resizeWindow("Depth Image", width, height)

        cv2_rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
        while(1):
            rotated_rgb = cv2.rotate(cv2_rgb_image, cv2.ROTATE_90_CLOCKWISE)
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

        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = RealSenseCamera()
    camera.capture_image()
    camera.visualize_image()