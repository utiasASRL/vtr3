import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.dates import date2num
import datetime
import argparse

def load_data(data_dir, run_ids):

    info = {}

    for k in run_ids:

        info[k] = {}

        for i in run_ids:

            if i != k:

                info[k][i] = {"timestamp":[],
                             "live_id":[],
                             "priv_id":[],
                             "success":[],
                             "inliers_rgb":[],
                             "inliers_gray":[],
                             "inliers_cc":[],
                             "window_temporal_depth":[],
                             "window_num_vertices":[],
                             "comp_time":[]}

                results_dir = "{}/teach_{}/graph.index/repeats/{}/results".format(data_dir, k, i)
                info_file_path = "{}/info.csv".format(results_dir) 

                with open(info_file_path) as csv_file:

                    csv_reader = csv.reader(csv_file, delimiter=',')
                    first = True

                    for row in csv_reader:

                        if not first:
                            info[k][i]["timestamp"] += [int(row[0])]
                            info[k][i]["live_id"] += [row[1]]
                            info[k][i]["priv_id"] += [row[2]]
                            info[k][i]["success"] += [row[3]]
                            info[k][i]["inliers_rgb"] += [float(row[4])]
                            info[k][i]["inliers_gray"] += [float(row[5])]
                            info[k][i]["inliers_cc"] += [float(row[6])]
                            info[k][i]["window_temporal_depth"] += [row[7]]
                            info[k][i]["window_num_vertices"] += [row[8]]
                            info[k][i]["comp_time"] += [float(row[9])]

                        first = False

                dt = datetime.datetime.fromtimestamp(info[k][i]["timestamp"][0] / 1e9) 
                print("{}-{}".format(i, dt.strftime('%d/%m-%H:%M')))

    return info

def compute_inliers(info):

    avg_inliers = {}
    failures = {}

    for teach_run in info.keys():

        avg_inliers[teach_run] = {}
        failures[teach_run] = {}

        for repeat_run in info[teach_run].keys():
    
            avg_inliers[teach_run][repeat_run] = \
                        sum(info[teach_run][repeat_run]["inliers_rgb"]) / \
                        float(len(info[teach_run][repeat_run]["inliers_rgb"]))

            fail = [0 if s == '1' else 1 for s in info[teach_run][repeat_run]["success"]]
            failures[teach_run][repeat_run] = sum(fail)
             
    print(avg_inliers)
    print(failures)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=None, type=str,
                        help='path to results dir (default: None)')

    args = parser.parse_args()

    run_ids = [8, 35, 174, 235, 538, 587]

    info = load_data(args.path, run_ids)

    compute_inliers(info);


