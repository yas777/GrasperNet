# import sys
# import os
# os.chdir("..")
# sys.path.append(os.path.abspath(os.curdir))
# print(sys.path)

from segment_anything import sam_model_registry, SamPredictor
from utils.seg_utils import visualize_masks
from global_parameters import SAM_CHECKPOINT, SAM_MODEL_TYPE

import cv2
import numpy as np

class Segment:
    def __init__(self, checkpoint, model_type):
        model = sam_model_registry[model_type](checkpoint=checkpoint)
        self.predictor = SamPredictor(model)

    def set_image(self, image, depth):
        self.predictor.set_image(image)
        self.depth_img = depth

    def segment_image(self, points, labels, get_end_points=True):
        """
            points -> np array of 2d points
            labels -> np array of labels for 2d points 1 -> foreground and 0 -> background

            Output:
            masks -> list of all possible masks for given prompts.
        """

        mask, score, _ = self.predictor.predict(
            point_coords=points,
            point_labels=labels,
            multimask_output=False,
        )

        print(mask.shape)
        if get_end_points:
            _, H, W = mask.shape
            px = points[0][0]
            # print(mask[0, :, px])

            mini_y_dep, maxi_y_dep, mini_y, maxi_y = 1000, 0, W, 0
            for y in range(H):
                if (mask[0, y, px]):
                    dep = self.depth_img[y][px]
                    print(y, dep)
                    if (dep):
                        if (y < mini_y):
                            mini_y = y
                            mini_y_dep = dep
                        elif (y > maxi_y):
                            maxi_y = y
                            maxi_y_dep = dep
        print(mini_y, maxi_y)
        return mask, score, mini_y, maxi_y 
    
if __name__ == '__main__':
    sam = Segment(SAM_CHECKPOINT, SAM_MODEL_TYPE)
    
    image = cv2.imread('./images/obj1.jpg')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    sam.set_image(image)

    input_point = np.array([[750, 1200]])
    input_label = np.array([1])

    masks, scores, mini_dep, maxi_dep = sam.segment_image(image, input_point, input_label)
    print(mini_dep, maxi_dep)
    visualize_masks(image, masks, scores, input_point, input_label)






    
