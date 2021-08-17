import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("TkAgg")
plt.ion()

# Load data
lidar = np.load("/ext0/datasets/electric_sheep/data0525/lidar.npy")
gps = np.load("/ext0/datasets/electric_sheep/data0525/gps_interpolated.npy")

# point cloud alignment
lidar_com = np.sum(lidar, axis=1, keepdims=True) / lidar.shape[1]
gps_com = np.sum(gps, axis=1, keepdims=True) / lidar.shape[1]

W = (lidar - lidar_com) @ (gps - gps_com).T / lidar.shape[1]
u, s, vh = np.linalg.svd(W)

I = np.eye(2)
I[1, 1] = np.linalg.det(u) * np.linalg.det(vh)
C = u @ I @ vh

# transform
lidar = C.T @ lidar + gps_com - C.T @ lidar_com

# plot
fig = plt.figure(1)
ax = fig.add_subplot()
ax.plot(lidar[0], lidar[1], c='r', label='LIDAR')
ax.plot(gps[0], gps[1], c='g', label='GPS')
plt.pause(10)