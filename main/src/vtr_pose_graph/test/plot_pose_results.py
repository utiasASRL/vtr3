import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import datetime
import argparse

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        table_names = self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';").fetchall()
        print(table_names)

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
    def get_bag_messages(self, topic_name):
        
        topic_id = self.topic_id[topic_name]

        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]   


def load_gps_poses(data_dir, num_repeats):

    gps_poses = []

    for i in range(num_repeats):

        gps_poses.append({"timestamp":[],
                          "latitude":[],
                          "longitude":[],
                          "altitude":[],
                          "cov": []})


        bag_file = '{}/navsatfix/run_00000{}/run_00000{}_0.db3'.format(data_dir, i + 1)
        
        parser = BagFileParser(bag_file)

        messages = parser.get_bag_messages("/fix") 

        for j in range(len(messages)):

            gps_msg = messages[j]

            gps_poses[i]["timestamp"] += [gps_msg[0]]
            gps_poses[i]["latitude"] += [gps_msg[1].latitude]
            gps_poses[i]["longitude"] += [gps_msg[1].longitude]
            gps_poses[i]["altitude"] += [gps_msg[1].altitude]
            gps_poses[i]["cov"] += [gps_msg[1].position_covariance[0]]
            
            if gps_msg[1].position_covariance[0] > 0.001:
                print("LARGE COVARIANCE: {} - run {}, node {}".format(gps_msg[1].position_covariance[0], i+1, j))

    return gps_poses


def load_est_poses(data_dir, num_repeats):

    est_poses = []

    for i in range(0, num_repeats):

        est_poses.append({"timestamp":[],
                          "live_id":[],
                          "priv_id":[],
                          "x":[],
                          "y":[],
                          "z":[],
                          "roll":[],
                          "pitch":[],
                          "yaw":[]})

        results_dir = "{}/graph.index/repeats/{}/results".format(data_dir, i+1)
        poses_file_path = "{}/poses.csv".format(results_dir) 

        with open(poses_file_path) as csv_file:

            csv_reader = csv.reader(csv_file, delimiter=',')
            first = True

            for row in csv_reader:

                if not first:
                    est_poses[i]["timestamp"] += [int(row[0])]
                    est_poses[i]["live_id"] += [row[1]]
                    est_poses[i]["priv_id"] += [row[2]]
                    est_poses[i]["x"] += [float(row[3])]
                    est_poses[i]["y"] += [float(row[4])]
                    est_poses[i]["z"] += [float(row[5])]
                    est_poses[i]["roll"] += [float(row[6])]
                    est_poses[i]["pitch"] += [float(row[7])]
                    est_poses[i]["yaw"] += [float(row[8])] 

                first = False


    return est_poses

def plot_comp_time(avg_comp_time, times, results_dir):
    
    plt.figure()
    plt.plot(avg_comp_time)
    plt.ylabel('Avg. computation time (ms)')
    plt.xlabel('Repeat (hh:mm)')
    plt.xticks(np.arange(len(times)), times) 
    plt.title('Avg. localization computation time for each run')

    plt.savefig('{}/avg_comp_time.png'.format(results_dir), format='png')
    plt.close()

def plot_inliers(avg_inliers, times, inliers, results_dir):

    # Plot average number of inliers for each run
    plt.figure()
    plt.plot(avg_inliers)
    plt.ylabel('Avg. number of inliers')
    plt.xlabel('Repeat (hh:mm)') 
    plt.xticks(np.arange(len(times)), times) 
    plt.title('Avg. number of inliers for each run')

    plt.savefig('{}/avg_inliers.png'.format(results_dir), format='png')
    plt.close()

    # Plot cumulative distribution of inliers for each run
    plt.figure(figsize=(16, 12)) #
    # plt.figure()
    # f.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot_lines = []
    labels = []

    for i in range(len(inliers)):
        
        max_val = np.max(inliers[i])
        n_bins_vis_range = 50
        n_bins_total = int((n_bins_vis_range * max_val) / 696)

        values, base = np.histogram(inliers[i], bins=n_bins_total)
        unity_values = values / values.sum()
        cumulative = np.cumsum(np.flip(unity_values,0))
        p = plt.plot(base[:-1], cumulative, linewidth=3)
        plot_lines.append(p[0])
        labels.append(times[i])

    plt.axvline(x=20.0, color='red', linewidth='3', linestyle='--')

    plt.legend(plot_lines, labels, prop={'size': 20})
    plt.xlim([696, 0])
    plt.ylim([0, 1])
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(True, which='both', axis='both', color='gray', linestyle='-', linewidth=1)
    plt.xlabel('Number of inliers', fontsize=20, weight='bold')
    plt.ylabel('CDF over keyframes', fontsize=20, weight='bold')
    plt.title('Distribution of keyframes with given number of inliers', fontsize=20, weight='bold')
    plt.savefig('{}/cumulative_dist_inliers.png'.format(results_dir), format='png')
    plt.close()

def plot_data(info, data_dir):

    avg_inliers = []
    inliers = []
    times = [] 
    avg_comp_time = []

    for i in range(len(info)):

        inliers.append(info[i]["inliers_rbg"])
        avg_inliers.append(sum(info[i]["inliers_rbg"]) / float(len(info[i]["inliers_rbg"])))
        avg_comp_time.append(sum(info[i]["comp_time"]) / float(len(info[i]["comp_time"])))

        dt = datetime.datetime.fromtimestamp(info[i]["timestamp"][0] / 1e9)	
        times.append(dt.strftime('%H:%M'))


    results_dir = "{}/graph.index/repeats".format(data_dir)

    plot_inliers(avg_inliers, times, inliers, results_dir)

    plot_comp_time(avg_comp_time, times, results_dir)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=None, type=str,
                        help='path to results dir (default: None)')
    parser.add_argument('--numrepeats', default=None, type=int,
                        help='number of repeats (default: None)')

    args = parser.parse_args()

    est_poses = load_est_poses(args.path, args.numrepeats)

    gps_poses = load_gps_poses(args.path, args.numrepeats)
   

    # plot_data(info, args.path);
