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


def load_gps_poses(data_dir, start, end):

    start_gps_coord = [0.0, 0.0, 0.0]
    start_xy_coord = [0.0, 0.0]
    gps_poses = []
    bad_gps = []
    plot_segments = {"x":{}, "y":{}}

    proj_origin = (43.7822845, -79.4661581, 169.642048)  # hardcoding for now - todo: get from ground truth CSV
    projection = Proj(
        "+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(proj_origin[0],
                                                                                               proj_origin[1],
                                                                                               proj_origin[2]))

    run_inds = [0] + list(range(start, end + 1))

    for i in run_inds:

        plot_segments["x"][i] = [[]]
        plot_segments["y"][i] = [[]]
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

        if i in bad_gps:
            continue

        try:
            parser = BagFileParser(bag_file)
        except Exception as e:
            print(e)
            print(i)
            if (i not in bad_gps):
                bad_gps.append(i)
            continue

        messages = parser.get_bag_messages("/fix") 

        # Was recording GPS when driving manually back into the garage. Discard
        # this messages.
        # if i == 15:
        #     messages = messages[:12250]

        # if i == 16:
        #     messages = messages[:12330]

        num_large_cov = 0

        for j in range(len(messages)):

            gps_msg = messages[j]

            x, y = projection(gps_msg[1].longitude, gps_msg[1].latitude)
            # x, y = gps_msg[1].longitude, gps_msg[1].latitude

            if (i == 0) and (j ==0):
                start_gps_coord = [gps_msg[1].latitude, 
                                   gps_msg[1].longitude,
                                   gps_msg[1].altitude]

                start_xy_coord = [x, y]

            if gps_msg[1].position_covariance[0] >= 0.05:
                num_large_cov += 1
                if not prev_vert_bad_gps:
                    plot_segments["x"][i] += [[]]
                    plot_segments["y"][i] += [[]]
                    segment_ind += 1
                prev_vert_bad_gps = True
                continue
            else:
                prev_vert_bad_gps = False

            gps_poses[-1]["timestamp"] += [gps_msg[0]]
            gps_poses[-1]["latitude"] += [gps_msg[1].latitude - start_gps_coord[0]]
            gps_poses[-1]["longitude"] += [gps_msg[1].longitude - start_gps_coord[1]]
            gps_poses[-1]["altitude"] += [gps_msg[1].altitude - start_gps_coord[2]]
            gps_poses[-1]["cov"] += [gps_msg[1].position_covariance[0]]
            gps_poses[-1]["x"] += [x - start_xy_coord[0]]
            gps_poses[-1]["y"] += [y - start_xy_coord[1]]

            plot_segments["x"][i][segment_ind] += [x - start_xy_coord[0]]
            plot_segments["y"][i][segment_ind] += [y - start_xy_coord[1]] 

        if num_large_cov > 0:
            print("Large cov {}: {}, total: {}".format(i, num_large_cov, len(messages)))
            if ((num_large_cov / len(messages)) > 0.7) and (i not in bad_gps):
                bad_gps.append(i)

    print("Bad GPS runs: {}".format(bad_gps))

    return gps_poses, bad_gps, plot_segments
   

def plot_data(gps_poses, errors, bad_gps, plot_segments, start, end, data_dir):

    results_dir = "{}/graph.index/repeats".format(data_dir)

    plt.figure(figsize=(22, 15))    
    plot_lines = []
    labels = []

    run_inds = [0] + list(range(start, end + 1))

    for i in run_inds:

        print("Num plot segments: {}".format(len(plot_segments["x"][i])))

        if i in bad_gps:
            continue

        p = None
        for j in range(len(plot_segments["x"][i])):
            p = plt.plot(plot_segments["x"][i][j], plot_segments["y"][i][j], linewidth=2)
            
        plot_lines.append(p[0]) # just grab the last one, just need one per path
        # abels.append(times[i].strftime('%H:%M'))
        labels.append(i)

        # p = plt.plot(gps_poses[i]["x"], gps_poses[i]["y"], linewidth=2)
        # plot_lines.append(p[0])
        # labels.append(i)

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

def compare_paths(gps_poses, bad_gps, start, end):

    errors = {}
    rms = {}
    teach_poses = gps_poses[0]

    # t_np = np.arange(0, len(teach_poses['x']))
    # x_np = np.array(teach_poses['x'])
    # y_np = np.array(teach_poses['y'])

    # x_tck = interpolate.splrep(t_np, x_np, s=0)
    # y_tck = interpolate.splrep(t_np, y_np, s=0)

    # t_np_new = np.arange(0, 3 * len(teach_poses['x']))
    # x_np_new = interpolate.splev(t_np_new, x_tck, der=0)
    # y_np_new = interpolate.splev(t_np_new, y_tck, der=0)


    ind = 1
    for i in range(start, end + 1):

        start = time.time()

        print(i)

        if i in bad_gps:
            continue

        errors[i] = []
        sum_sqr_error = 0.0

        repeat_poses = gps_poses[ind]

        latest_match_ind = 0
        max_match_ind = len(teach_poses['x'])

        for j in range(len(repeat_poses['x'])):

            repeat_pose = np.array([repeat_poses['x'][j],
                                    repeat_poses['y'][j]])

            # Match to a teach pose
            # match_ind = latest_match_ind - 1000
            # if match_ind < 0:
            #     match_ind = 0

            match_ind = 0

            min_dist_ind = match_ind
            min_dist = 100000000000.0
            max_match_ind_loop = max_match_ind # if j==0 else latest_match_ind + 1000
            
            while (match_ind < max_match_ind_loop) and (match_ind < max_match_ind):
                teach_pose = np.array([teach_poses['x'][match_ind],
                                       teach_poses['y'][match_ind]])

                # teach_pose = np.array([x_np_new[match_ind],
                #                        y_np_new[match_ind]])

                dist = np.linalg.norm(repeat_pose - teach_pose)
                if dist < min_dist:
                    min_dist = dist
                    min_dist_ind = match_ind

                match_ind +=1

            errors[i] = errors[i] + [min_dist]
            latest_match_ind = min_dist_ind
            sum_sqr_error += (min_dist * min_dist)

        rms[i] = math.sqrt(sum_sqr_error / len(repeat_poses['x']))

        print("RMS: {}".format(rms))
        print(time.time()-start)
        ind += 1

    return errors, rms


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    
    # Assuming following path structure:
    # vtr_folder/graph.index
    # vtr_folder/navsatfix/run_000xxx/metadata.yaml
    # vtr_folder/navsatfix/run_000xxx/run_000xxx_0.db3
    parser.add_argument('--path', default=None, type=str,
                        help='path to vtr folder (default: None)')
    parser.add_argument('--start', default=None, type=int,
                        help='first repeat (default: None)')
    parser.add_argument('--end', default=None, type=int,
                        help='last repeat (default: None)')

    args = parser.parse_args()

    gps_poses, bad_gps, plot_segments = load_gps_poses(args.path, args.start, args.end)  

    # ignore_runs = [5, 6, 14, 15, 16] #extended

    # bad_gps = bad_gps + ignore_runs

    errors, rms = compare_paths(gps_poses, bad_gps,args.start, args.end) 

    results_dir = "{}/graph.index/repeats".format(args.path)
    pickle.dump(errors, open( "{}/gps_errors_{}_{}.p".format(results_dir, args.start, args.end), "wb"))
    pickle.dump(rms, open( "{}/gps_rms_{}_{}.p".format(results_dir, args.start, args.end), "wb"))

    # errors = {}

    # ignore_runs = [9,17, 18, 23] #extended

    # bad_gps = bad_gps + ignore_runs

    plot_data(gps_poses, errors, bad_gps, plot_segments, args.start, args.end, args.path);
