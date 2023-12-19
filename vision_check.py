import numpy as np
import open3d as o3d
import cv2
from segment import segment_image

from utils import plane_detection, segment_point_cloud

if __name__ == "__main__":
    rgb_image = np.load("rgb.npy")
    depth_image = np.load("depth.npy")
    points = np.load("points.npy")

    # seg_mask = np.load("seg_mask.npy").view(np.bool_)
    # bbox = np.load("pred_boxes.npy")
    
    # h, w, _ = rgb_image.shape

    # predictions = segment_image("box")
    # # bbox = predictions['instances'].pred_boxes[0].tensor.numpy()
    # # print(bbox)
    # # seg_mask = predictions['instances'].pred_masks[0].numpy()
    # # print(seg_mask)
    # # np.save("seg_mask.npy", seg_mask)
    # # np.save("pred_boxes.npy", bbox)
    # # print(predictions)

    # xmin, ymin, xmax, ymax = int(bbox[0][0]), int(bbox[0][1]), int(bbox[0][2]), int(bbox[0][3])
    # print(xmin, ymin, xmax, ymax)

    # crop_rgb_image = rgb_image[ymin:ymax, xmin:xmax]
    # crop_depth_image = depth_image[ymin:ymax, xmin:xmax]
    # crop_points = points[ymin:ymax, xmin:xmax]
    # crop_seg_mask = seg_mask[ymin:ymax, xmin:xmax]

    # seg_3d_mask = np.dstack((crop_seg_mask, crop_seg_mask, crop_seg_mask))
    # crop_points1 = np.zeros(crop_points.shape)
    # crop_points1[:,:,0:2] = crop_points[:,:,0:2]
    # crop_points = np.where(np.invert(seg_3d_mask), 0, crop_points)
    # # print(points)
    # print(seg_mask.sum(), crop_seg_mask.sum(), crop_seg_mask.shape)
    # # print(rgb_image/255)
    # # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    # # images = np.hstack((rgb_image, depth_colormap))
            
    # # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
    # # cv2.imshow('Align Example', images)
    # # cv2.waitKey(10000)

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(crop_points.reshape(-1, 3))
    # pcd.colors = o3d.utility.Vector3dVector((crop_rgb_image/255).reshape(-1, 3))

    # print(len(pcd.points))

    pcd = segment_point_cloud(rgb_image, depth_image, points, "box")
    print(len(pcd.points))
    pcd = pcd.voxel_down_sample(voxel_size=0.001)
    print(len(pcd.points))
    
    plane_detection(pcd, vis=True)
    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(pcd)
    # vis.run()