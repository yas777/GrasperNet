import rospy
import numpy as np
from PIL import Image as PILImage

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray,MultiArrayDimension


IMAGE_PUBLISHER_NAME = '/gopro_image'
DEPTH_PUBLISHER_NAME = '/gopro_depth'

def convert_numpy_array_to_float32_multi_array(matrix):
	# Create a Float64MultiArray object
    data_to_send = Float32MultiArray()

    # Set the layout parameters
    data_to_send.layout.dim.append(MultiArrayDimension())
    data_to_send.layout.dim[0].label = "rows"
    data_to_send.layout.dim[0].size = len(matrix)
    data_to_send.layout.dim[0].stride = len(matrix) * len(matrix[0])

    data_to_send.layout.dim.append(MultiArrayDimension())
    data_to_send.layout.dim[1].label = "columns"
    data_to_send.layout.dim[1].size = len(matrix[0])
    data_to_send.layout.dim[1].stride = len(matrix[0])

    # Flatten the matrix into a list
    data_to_send.data = matrix.flatten().tolist()

    return data_to_send

class ImagePublisher():

    def __init__(self, camera):

        self.camera = camera

        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher(IMAGE_PUBLISHER_NAME, Image, queue_size = 1)
        self.depth_publisher = rospy.Publisher(DEPTH_PUBLISHER_NAME, Float32MultiArray, queue_size = 1)

    def publish_image(self):

        image, depth, points = self.camera.capture_image()

        rotated_image = np.rot90(image, k=-1)
        rotated_depth = np.rot90(depth, k=-1)

        PILImage.fromarray(rotated_image).save("./images/test_rgb9.png")
        PILImage.fromarray(rotated_depth).save("./images/test_depth9.png")
        # np.save("./images/test_rgb.npy", rotated_image)
        # np.save("./images/test_depth.npy", rotated_depth)

        try:
            image_message = self.bridge.cv2_to_imgmsg(rotated_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        depth_data = convert_numpy_array_to_float32_multi_array(rotated_depth)
        print(image_message.shape)
        print(depth_data.shape)
