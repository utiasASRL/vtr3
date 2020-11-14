#!/usr/bin/env python

import csv
import os.path as osp
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

matplotlib.use("TkAgg")  # Can change to 'Agg' for non-interactive mode
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42


def read_vo(vo_dir):
  with open(osp.join(vo_dir, "vo.csv"), newline='') as resultfile:
    spamreader = csv.reader(resultfile, delimiter=',', quotechar='|')
    tmp = []
    for i, row in enumerate(spamreader):
      if i == 0:
        continue
      else:
        tmp.append([float(i) for i in row[3:6]])
        assert len(tmp[-1]) == 3
  return np.array(tmp)


def read_vo_transforms(data_dir):
  """Extract integrated VO transformations from teach run. Used to transform loc results"""
  vo_transforms = {}

  with open(osp.join(data_dir, "vo.csv"), newline='') as resultfile:
    spamreader = csv.reader(resultfile, delimiter=',', quotechar='|')

    for i, row in enumerate(spamreader):
      if i == 0:
        continue
      else:
        tmp = [float(i) for i in row[6:]]
        assert len(tmp) == 16
        vo_transforms[int(row[2])] = np.array(tmp).reshape((4, 4), order='F')    # csv is column-major

  return vo_transforms


def read_loc(teach_dir, repeat_dir):
  """Robot position from composing integrated VO and localization"""

  vo_transforms = read_vo_transforms(teach_dir)

  with open(osp.join(repeat_dir, "loc.csv"), newline='') as resultfile:
    spamreader = csv.reader(resultfile, delimiter=',', quotechar='|')
    r_worlds = np.empty((0, 4))
    for i, row in enumerate(spamreader):
      if i == 0:
        continue
      else:
        r_loc = np.empty([4, 1])
        r_loc[0] = float(row[5])
        r_loc[1] = float(row[6])
        r_loc[2] = float(row[7])
        r_loc[3] = 1.0

        map_T = vo_transforms[int(row[4])]
        r_world = np.matmul(map_T, r_loc)
        r_worlds = np.append(r_worlds, r_world.T, axis=0)

  return r_worlds


def main():

  # set sizes
  SMALL_SIZE = 10
  MEDIUM_SIZE = 12
  BIGGER_SIZE = 16
  plt.rc("font", size=MEDIUM_SIZE)  # controls default text sizes
  plt.rc("axes", titlesize=MEDIUM_SIZE)  # fontsize of the axes title
  plt.rc("axes", labelsize=SMALL_SIZE)  # fontsize of the x and y labels
  plt.rc("xtick", labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
  plt.rc("ytick", labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
  plt.rc("figure", titlesize=MEDIUM_SIZE)  # fontsize of the figure title
  plt.rc("legend", fontsize=SMALL_SIZE)  # legend fontsize

  # Flags
  teach_dir = osp.expanduser("~/ASRL/vtr3_offline_test/results/run_000000")
  repeat_dir = osp.expanduser("~/ASRL/vtr3_offline_test/results/run_000001")

  r_teach = read_vo(teach_dir)
  r_repeat = read_loc(teach_dir, repeat_dir)

  print("Number of teach points: ", r_teach.shape[0])
  print("Number of repeat points: ", r_repeat.shape[0])

  fig = plt.figure()
  ax = fig.add_subplot(111)
  ax.plot(r_teach[:, 0], r_teach[:, 1])
  ax.plot(r_repeat[:, 0], r_repeat[:, 1])
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  plt.show()


if __name__ == '__main__':
    main()
