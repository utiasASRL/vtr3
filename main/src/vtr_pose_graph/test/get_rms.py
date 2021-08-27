import csv
import numpy as np
from scipy import interpolate
import matplotlib
import matplotlib.pyplot as plt
import datetime
import argparse
from pyproj import Proj
import time
import pickle
import math

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    
    # Assuming following path structure:
    # vtr_folder/graph.index
    # vtr_folder/navsatfix/run_000xxx/metadata.yaml
    # vtr_folder/navsatfix/run_000xxx/run_000xxx_0.db3
    parser.add_argument('--path', default=None, type=str,
                        help='path to vtr folder (default: None)')

    args = parser.parse_args()

    results_dir = "{}/graph.index/repeats".format(args.path)
    # pickle.dump(errors, open( "{}/gps_errors_{}_{}.p".format(results_dir, args.start, args.end), "wb"))
    # pickle.dump(rms, open( "{}/gps_rms_{}_{}.p".format(results_dir, args.start, args.end), "wb"))

    files = ['gps_errors_0_10.p', 'gps_errors_11_20.p', 'gps_errors_21_30.p', 'gps_errors_31_40.p']

    errors_all_runs = []

    for name in files:
        file_path = "{}/{}".format(results_dir, name)

        with open(file_path, "rb") as input_file:
            e = pickle.load(input_file)

            for key in e.keys():
                errors_all_runs += e[key]

    sum_sqr_error = 0.0

    print(errors_all_runs)
    print(len(errors_all_runs))

    max_err = 0
    num = len(errors_all_runs)

    for i in range(len(errors_all_runs)):
        if errors_all_runs[i] > 1.0:
            print(errors_all_runs[i])
            num -= 1
            continue
        # print(errors_all_runs[i] * errors_all_runs[i])
        sum_sqr_error += (errors_all_runs[i] * errors_all_runs[i])

    print(num)
    print(len(errors_all_runs))
    
    print(sum_sqr_error)
    rms = math.sqrt(sum_sqr_error / num)

    print("Total RMS: {}".format(rms))
