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
    bad_gps_runs = []
    gps_good_segments = {"x":{}, "y":{}}

    proj_origin = (43.7822845, -79.4661581, 169.642048)  # hardcoding for now - todo: get from ground truth CSV
    projection = Proj(
        "+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(proj_origin[0],
                                                                                               proj_origin[1],
                                                                                               proj_origin[2]))

    for i in range(num_repeats + 1):

        # Collecting segements along a path that have good gps so we can plot a 
        # list of good segmants and ignore the bad data.
        gps_good_segments["x"][i] = [[]]
        gps_good_segments["y"][i] = [[]]
        segment_ind = 0
        prev_vert_bad_gps = False

        print("Run: {}".format(i))

        gps_poses.append({"timestamp":[],
                          "latitude":[],
                          "longitude":[],
                          "altitude":[],
                          "cov": [],
                          "x":[],
                          "y":[]})


        bag_file = '{}/navsatfix/run_{}/run_{}_0.db3'.format(data_dir, str(i).zfill(6), str(i).zfill(6))

        try:
            parser = BagFileParser(bag_file)
        except Exception as e:
            print("Could not open rosbag for run {}".format(i))
            continue

        messages = parser.get_bag_messages("/fix") 

        num_large_covariance = 0

        for j in range(len(messages)):

            gps_msg = messages[j]

            # Projecting GPS so we can plot in meters.
            x, y = projection(gps_msg[1].longitude, gps_msg[1].latitude)
            # x, y = gps_msg[1].longitude, gps_msg[1].latitude

            if (i == 0) and (j ==0):
                # Want to plot with the start of the path at (0, 0)
                start_gps_coord = [gps_msg[1].latitude, 
                                   gps_msg[1].longitude,
                                   gps_msg[1].altitude]

                start_xy_coord = [x, y]

            # Check if covariance is good. Up to 0.1 is ok, I've chosen slightly
            # stricter setting here.
            covariance_is_large = gps_msg[1].position_covariance[0] >= 0.05
            if covariance_is_large:
                num_large_covariance += 1
                
                # Start a new segment of good GPS data that can be plotted.
                if not prev_vert_bad_gps:
                    gps_good_segments["x"][i] += [[]]
                    gps_good_segments["y"][i] += [[]]
                    segment_ind += 1
                prev_vert_bad_gps = True
            else:
                prev_vert_bad_gps = False

            gps_poses[i]["timestamp"] += [gps_msg[0]]
            gps_poses[i]["latitude"] += [gps_msg[1].latitude - start_gps_coord[0]]
            gps_poses[i]["longitude"] += [gps_msg[1].longitude - start_gps_coord[1]]
            gps_poses[i]["altitude"] += [gps_msg[1].altitude - start_gps_coord[2]]
            gps_poses[i]["cov"] += [gps_msg[1].position_covariance[0]]
            gps_poses[i]["x"] += [x - start_xy_coord[0]]
            gps_poses[i]["y"] += [y - start_xy_coord[1]]

            if not covariance_is_large:
                gps_good_segments["x"][i][segment_ind] += [x - start_xy_coord[0]]
                gps_good_segments["y"][i][segment_ind] += [y - start_xy_coord[1]] 

        if num_large_covariance > 0:
            print("Large cov {}: {}, total: {}".format(i, num_large_covariance, len(messages)))
            if i not in bad_gps_runs:
                bad_gps_runs.append(i)

    print("Bad GPS runs: {}".format(bad_gps_runs))

    return gps_poses, bad_gps_runs, gps_good_segments
   

def plot_data(gps_poses, gps_good_segments, bad_gps_runs, plot_good_gps_only, data_dir):

    plt.figure(figsize=(22, 15))    
    plot_lines = []
    labels = []

    for i in range(len(gps_poses)):

        if (i in bad_gps_runs) and plot_good_gps_only:
            continue

        if plot_good_gps_only:
            p = None
            for j in range(len(gps_good_segments["x"][i])):
                p = plt.plot(gps_good_segments["x"][i][j], gps_good_segments["y"][i][j], linewidth=2, color='C{}'.format(i))
        else:
            p = plt.plot(gps_poses[i]["x"], gps_poses[i]["y"], linewidth=2, color='C{}'.format(i))

        plot_lines.append(p[0])
        labels.append(i)

    plt.ylabel('y (m)', fontsize=20, weight='bold')
    plt.xlabel('x (m)', fontsize=20, weight='bold') 
    plt.xticks(fontsize=20) 
    plt.yticks(fontsize=20) 
    plt.title('GPS ground truth, teach and repeat runs', fontsize=22, weight='bold')
    plt.legend(plot_lines, labels, fontsize=12)

    plt.savefig('{}/gps_paths.png'.format(data_dir), bbox_inches='tight', format='png')
    plt.close()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    
    # Assuming following path structure:
    # vtr_folder/navsatfix/run_000xxx/metadata.yaml
    # vtr_folder/navsatfix/run_000xxx/run_000xxx_0.db3
    parser.add_argument('--path', default=None, type=str,
                        help='path to vtr folder (default: None)')
    parser.add_argument('--numrepeats', default=None, type=int,
                        help='number of repeats (default: None)')
    # Add argument if we want to only plot segements of the path with good GPS.
    parser.add_argument('--only_good', dest='only_good', action='store_true')

    args = parser.parse_args()

    gps_poses, gps_good_segments, bad_gps_runs = load_gps_poses(args.path, args.numrepeats)    

    plot_data(gps_poses, bad_gps_runs, gps_good_segments, args.only_good, args.path);
