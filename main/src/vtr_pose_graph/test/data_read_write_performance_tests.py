# Copyright 2021, Autonomous Space Robotics Lab (ASRL)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Execute this file in the result directory of read_write_performance_tests
"""

import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

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

entries = ["data_size", "read_time", "write_time"]
unit = ["(Mb)", "(ms)", "(ms)"]
header = None
result = {}

# Load results, divide by 1e6 to milliseconds and Mb
for k in entries:
  with open(k + ".csv", newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    tmp = []
    for i, row in enumerate(spamreader):
      if i == 0:
        header = np.array([float(i) for i in row[1:-1]])
      else:
        tmp.append([float(i) for i in row[1:-1]])
  header = np.array(header)
  result[k] = np.array(tmp)
  result[k + "_mean"] = np.mean(np.array(tmp), axis=0) / 1e6
  result[k + "_std"] = np.std(np.array(tmp), axis=0) / 1e6
  result[k + "_avg_mean"] = np.mean(np.array(tmp) / header[None,], axis=0) / 1e6
  result[k + "_avg_std"] = np.std(np.array(tmp) / header[None,], axis=0) / 1e6

print(header, result)

fig = plt.figure()
size = 5
fig.set_size_inches(size * len(entries), size * 2)
fig.subplots_adjust(left=0.15,
                    right=0.9,
                    bottom=0.25,
                    top=0.95,
                    wspace=0.25,
                    hspace=0.25)
fig.clf()

# Plot mean
for i, k in enumerate(entries):
  ax = fig.add_subplot(2, len(entries), i + 1)
  ax.plot(header, result[k + "_mean"])
  ax.fill_between(header,
                  result[k + "_mean"] - 1.0 * result[k + "_std"],
                  result[k + "_mean"] + 1.0 * result[k + "_std"],
                  alpha=0.2)
  #
  ax.set_xlabel("Number of Data")
  ax.set_ylabel(k + unit[i])
  ax.set_xscale("log")
  ax.set_yscale("log")

  ax.tick_params(axis="x", pad=5, length=5, width=1)
  ax.tick_params(axis="y", pad=5, length=5, width=1)
  # ax.ticklabel_format(style="sci", scilimits=(-2, 2), axis="both")

  ax.spines["top"].set_visible(False)
  ax.spines["right"].set_visible(False)
  ax.spines["bottom"].set_visible(True)
  ax.spines["left"].set_visible(True)

  # use ax level legend
  # ax.legend(loc="upper left", frameon=False)

# Plot mean (divide by data)
for i, k in enumerate(entries):
  ax = fig.add_subplot(2, len(entries), i + 1 + len(entries))
  ax.plot(header, result[k + "_avg_mean"])
  ax.fill_between(header,
                  result[k + "_avg_mean"] - 1.0 * result[k + "_avg_std"],
                  result[k + "_avg_mean"] + 1.0 * result[k + "_avg_std"],
                  alpha=0.2)
  #
  ax.set_xlabel("Number of Data")
  ax.set_ylabel(k + "/num_of_data" + unit[i])
  ax.set_xscale("log")
  # ax.set_yscale("log")

  ax.tick_params(axis="x", pad=5, length=5, width=1)
  ax.tick_params(axis="y", pad=5, length=5, width=1)
  ax.ticklabel_format(style="sci", scilimits=(-2, 2), axis="y")

  ax.spines["top"].set_visible(False)
  ax.spines["right"].set_visible(False)
  ax.spines["bottom"].set_visible(True)
  ax.spines["left"].set_visible(True)

  # use ax level legend
  # ax.legend(loc="upper left", frameon=False)

plt.show()