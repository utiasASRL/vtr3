import numpy as np
import open3d as o3d

from pyboreas.data.sequence import Sequence

from vtr_lidar.segmentation.himmelsbach import Himmelsbach

seq = Sequence("/ext0/datasets/boreas/sequences", ["boreas-2020-12-01-13-26"])


def remove_ground_himmelsbach(points, /, show=False):

  himmel = Himmelsbach(points)
  himmel.set_alpha(2.0 / 180.0 * np.pi)
  himmel.set_tolerance(0.25)
  himmel.set_thresholds(0.4, 0.2, 0.8, 5, 5)
  print("Himmelsbach initialized")
  ground_idx = himmel.compute_model_and_get_inliers()

  if show:  # Plot inliers/outliers result of plane fit
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    ground_cloud = pcd.select_by_index(ground_idx)
    ground_cloud.paint_uniform_color([0, 0, 0])
    object_cloud = pcd.select_by_index(ground_idx, invert=True)

    o3d.visualization.draw_geometries([ground_cloud, object_cloud], window_name='GP Removal Results (Black = Ground)')

  return ground_idx


frame = seq.lidar_frames[0]
points = frame.load_data()
ground_idx = remove_ground_himmelsbach(points, show=True)