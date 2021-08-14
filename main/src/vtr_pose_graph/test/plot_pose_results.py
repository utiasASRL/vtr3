import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import datetime
import argparse
from pyproj import Proj

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

class BagFileParser():
    def __init__(self, bag_file):
        try:
            self.conn = sqlite3.connect(bag_file)
        except Exception as e:
            print(e)
            print('could not connect')
            raise Exception('could not connect')

        self.cursor = self.conn.cursor()

        table_names = self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';").fetchall()

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

    start_gps_coord = [0.0, 0.0, 0.0]
    start_xy_coord = [0.0, 0.0]
    gps_poses = []
    bad_gps = [6,7,8,9]

    # proj_origin = (43.7822845, -79.4661581, 169.642048)  # hardcoding for now - todo: get from ground truth CSV
    # projection = Proj(
    #     "+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(proj_origin[0],
    #                                                                                            proj_origin[1],
    #                                                                                            proj_origin[2]))

    for i in range(num_repeats + 1):

        gps_poses.append({"timestamp":[],
                          "latitude":[],
                          "longitude":[],
                          "altitude":[],
                          "cov": [],
                          "x":[],
                          "y":[]})


        bag_file = '{}/navsatfix/run_{}/run_{}_0.db3'.format(data_dir, str(i).zfill(6), str(i).zfill(6))

        if i in bad_gps:
            continue

        try:
            parser = BagFileParser(bag_file)
        except Exception as e:
            print(e)
            print(i)
            continue

        messages = parser.get_bag_messages("/fix") 

        num_large_cov = 0

        for j in range(len(messages)):

            gps_msg = messages[j]

            # x, y = projection(gps_msg[1].longitude, gps_msg[1].latitude)
            x, y = gps_msg[1].longitude, gps_msg[1].latitude

            if (i == 0) and (j ==0):
                start_gps_coord = [gps_msg[1].latitude, 
                                   gps_msg[1].longitude,
                                   gps_msg[1].altitude]

                start_xy_coord = [x, y]

            gps_poses[i]["timestamp"] += [gps_msg[0]]
            gps_poses[i]["latitude"] += [gps_msg[1].latitude - start_gps_coord[0]]
            gps_poses[i]["longitude"] += [gps_msg[1].longitude - start_gps_coord[1]]
            gps_poses[i]["altitude"] += [gps_msg[1].altitude - start_gps_coord[2]]
            gps_poses[i]["cov"] += [gps_msg[1].position_covariance[0]]
            gps_poses[i]["x"] += [x - start_xy_coord[0]]
            gps_poses[i]["y"] += [y - start_xy_coord[1]]
            
            if gps_msg[1].position_covariance[0] >= 0.1:
                num_large_cov += 1

        if num_large_cov > 0:
            print("Large cov {}: {}, total: {}".format(i, num_large_cov, len(messages)))
            # if i not in bad_gps:
            #     bad_gps.append(i)

    print("Bad GPS runs: {}".format(bad_gps))

    return gps_poses, bad_gps
   

def plot_data(gps_poses, errors, bad_gps, data_dir):

    results_dir = "{}/graph.index/repeats".format(data_dir)

    plt.figure(figsize=(22, 15))    
    plot_lines = []
    labels = []

    for i in range(len(gps_poses)):

        if i in bad_gps:
            continue

        # if i not in [0, 27]:
        #     continue

        # p = plt.plot(gps_poses[i]["longitude"], gps_poses[i]["latitude"], linewidth=2)
        p = plt.plot(gps_poses[i]["x"], gps_poses[i]["y"], linewidth=2)
        plot_lines.append(p[0])
        labels.append(i)

    plt.ylabel('y (m)', fontsize=20, weight='bold')
    plt.xlabel('x (m)', fontsize=20, weight='bold') 
    plt.xticks(fontsize=20) 
    plt.yticks(fontsize=20) 
    plt.title('GPS ground truth, teach and repeat runs', fontsize=22, weight='bold')
    plt.legend(plot_lines, labels, fontsize=12)


    plt.savefig('{}/gps_paths.png'.format(results_dir), bbox_inches='tight', format='png')
    plt.close()

    plt.figure(figsize=(22, 15))    
    plot_lines = []
    labels = []

    for key in errors.keys():

        p = plt.plot(errors[key])
        plot_lines.append(p[0])
        labels.append(key)

    plt.ylabel('Distance', fontsize=20, weight='bold')
    plt.xlabel('Node', fontsize=20, weight='bold') 
    plt.xticks(fontsize=20) 
    plt.yticks(fontsize=20) 
    plt.title('Teach and repeat path runs, errors', fontsize=22, weight='bold')
    plt.legend(plot_lines, labels, fontsize=16)


    plt.savefig('{}/gps_errors.png'.format(results_dir), format='png')
    plt.close()

def compare_paths(gps_poses, bad_gps):

    errors = {}
    teach_poses = gps_poses[0]

    for i in range(len(gps_poses) - 1):
    # for i in range(5 - 1):

        print(i)

        if i in bad_gps:
            continue

        errors[i] = []

        repeat_poses = gps_poses[i+1]

        latest_match_ind = 0
        max_match_ind = len(teach_poses['x'])

        for j in range(len(repeat_poses['x'])):

            repeat_pose = np.array([repeat_poses['x'][j],
                                    repeat_poses['y'][j]])

            # Match to a teach pose
            match_ind = 0           
            min_dist_ind = match_ind
            min_dist = 100000000000.0
            max_match_ind_loop = max_match_ind if j==0 else latest_match_ind + 50
            
            while match_ind < max_match_ind:
                teach_pose = np.array([teach_poses['x'][match_ind],
                                       teach_poses['y'][match_ind]])

                dist = np.linalg.norm(repeat_pose - teach_pose)
                if dist < min_dist:
                    min_dist = dist
                    min_dist_ind = match_ind

                match_ind +=1

            errors[i] = errors[i] + [min_dist]
            latest_match_ind = min_dist_ind

    return errors


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=None, type=str,
                        help='path to results dir (default: None)')
    parser.add_argument('--numrepeats', default=None, type=int,
                        help='number of repeats (default: None)')

    args = parser.parse_args()

    gps_poses, bad_gps = load_gps_poses(args.path, args.numrepeats)  

    # errors = compare_paths(gps_poses, bad_gps) 

    errors = {}

    plot_data(gps_poses, errors, bad_gps, args.path);
