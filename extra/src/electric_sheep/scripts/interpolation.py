from collections import OrderedDict
import numpy as np


def find_nearest(array, value):
  """Find nearest indices of value a, b and their linear interpolation weights
    """
  array = np.asarray(array)
  idx = (np.abs(array - value)).argmin()
  if idx + 1 < len(array) and array[idx] <= value and array[idx + 1] >= value:
    a = array[idx]
    b = array[idx + 1]
  elif idx > 0 and array[idx] >= value and array[idx - 1] <= value:
    a = array[idx - 1]
    b = array[idx]
  else:
    return None, None, None, None
  c = value
  return a, b, (b - c) / (b - a), (c - a) / (b - a)


def interpolate_gps_at_lidar_time(gps_file="gps.npy",
                                  lidar_time_file="lidarstamp.npy"):

  gps = np.load("gps.npy")
  lidar_time = np.load("lidarstamp.npy")

  gps_dict = OrderedDict()
  for i in range(gps.shape[1]):
    gps_dict[gps[0, i]] = np.array([gps[1, i], gps[2, i]])
  gps_time = np.array(list(gps_dict.keys()))

  lidar_dict = OrderedDict()
  lidar_array = np.empty((lidar_time.shape[0], 3))
  for i in range(lidar_time.shape[0]):
    a, b, aw, bw = find_nearest(gps_time, lidar_time[i])
    lidar_dict[lidar_time[i]] = aw * gps_dict[a] + bw * gps_dict[b]
    lidar_array[i, 0] = lidar_time[i]
    lidar_array[i, 1] = lidar_dict[lidar_time[i]][0]
    lidar_array[i, 2] = lidar_dict[lidar_time[i]][1]

  np.save("interpolated_lidar.npz", lidar_array)


if __name__ == "__main__":
  interpolate_gps_at_lidar_time()
