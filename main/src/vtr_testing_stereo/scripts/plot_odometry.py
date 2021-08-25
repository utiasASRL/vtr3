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


def main():

  # Flags
  results_dir = osp.expanduser(osp.expandvars("${VTRDATA}/experiments/2021-08-14/lighting-change-surf/results_run_000000"))

  entries = ["data_size", "read_time", "write_time"]
  unit = ["(Mb)", "(ms)", "(ms)"]
  header = None
  result = {}

  with open(osp.join(results_dir, "vo.csv"), newline='') as resultfile:
    spamreader = csv.reader(resultfile, delimiter=',', quotechar='|')
    tmp = []
    for i, row in enumerate(spamreader):
      if i == 0:
        continue
      else:
        tmp.append([float(i) for i in row[3:6]])
        assert len(tmp[-1]) == 3

  r = np.array(tmp)
  print("Number of points: ", r.shape[0])

  fig = plt.figure()
  ax = fig.add_subplot(111)
  ax.plot(r[:, 0], r[:, 1])
  plt.axis('equal')
  plt.title("Integrated VO")
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.show()


if __name__ == '__main__':
  main()
