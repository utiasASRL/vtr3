#!/usr/bin/env python

import csv
import os.path as osp
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

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
    r_qm_in_ms = np.empty((0, 4))
    for i, row in enumerate(spamreader):
      if i == 0:
        continue
      else:
        r_loc = np.empty([4, 1])
        r_loc[0] = float(row[6])
        r_loc[1] = float(row[7])
        r_loc[2] = float(row[8])
        r_loc[3] = 1.0

        map_T = vo_transforms[int(row[4])]
        r_world = np.matmul(map_T, r_loc)
        r_worlds = np.append(r_worlds, r_world.T, axis=0)
        r_qm_in_ms = np.append(r_qm_in_ms, r_loc.T, axis=0)

  return r_worlds, r_qm_in_ms


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
  teach_dir = osp.expanduser("~/ASRL/vtr3_offline_test/results/")
  repeat_dir = osp.expanduser("~/ASRL/vtr3_offline_test/results/")

  r_teach = read_vo(teach_dir + "run_000000")

  r_repeat = {}
  r_qm = {}

  i = 1
  while osp.exists(repeat_dir + "run_" + str(i).zfill(6)):
    r_repeat[i], r_qm[i] = read_loc(teach_dir + "run_000000", repeat_dir + "run_" + str(i).zfill(6))
    i = i + 1

  print("Number of teach points: ", r_teach.shape[0])
  print("Number of points in first repeat: ", r_repeat[1].shape[0])

  fig = plt.figure(1)
  ax = fig.add_subplot(111)
  plt.axis('equal')
  ax.plot(r_teach[:, 0], r_teach[:, 1], label='Teach')

  for j, run in r_repeat.items():
    ax.plot(run[:, 0], run[:, 1], label='Repeat ' + str(j))

  plt.title('Overhead View of Integrated VO and Localization')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  plt.legend()

  fig = plt.figure(2)
  latest = max(r_qm, key=int)
  plt.plot(r_qm[latest][:, 0], label='x')
  plt.plot(r_qm[latest][:, 1], label='y')
  plt.plot(r_qm[latest][:, 2], label='z')
  # for j, run in r_qm.items():
  #   plt.plot(run[:, 0], label='x_'+str(j), c='C0')      # plots all errors but messy
  # plt.plot(run[:, 1], label='y_'+str(j), c='C1')
  # plt.plot(run[:, 2], label='z_'+str(j), c='C2')
  plt.title('Estimated Path-Tracking Error')
  plt.ylabel('Distance (m)')
  plt.legend()

  plt.show()


if __name__ == '__main__':
    main()
