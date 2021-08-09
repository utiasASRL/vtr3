import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import datetime
import argparse

def load_data(data_dir, num_repeats):

    info = []

    for i in range(0, num_repeats):

        info.append({"timestamp":[],
                     "live_id":[],
                     "priv_id":[],
                     "success":[],
                     "inliers_rgb":[],
                     "inliers_gray":[],
                     "inliers_cc":[],
                     "window_temporal_depth":[],
                     "window_num_vertices":[],
                     "comp_time":[]})

        results_dir = "{}/graph.index/repeats/{}/results".format(data_dir, i+1)
        info_file_path = "{}/info.csv".format(results_dir) 

        with open(info_file_path) as csv_file:

            csv_reader = csv.reader(csv_file, delimiter=',')
            first = True

            for row in csv_reader:

                if not first:
                    info[i]["timestamp"] += [int(row[0])]
                    info[i]["live_id"] += [row[1]]
                    info[i]["priv_id"] += [row[2]]
                    info[i]["success"] += [row[3]]
                    info[i]["inliers_rgb"] += [float(row[4])]
                    info[i]["inliers_gray"] += [float(row[5])]
                    info[i]["inliers_cc"] += [float(row[6])]
                    info[i]["window_temporal_depth"] += [row[7]]
                    info[i]["window_num_vertices"] += [row[8]]
                    info[i]["comp_time"] += [float(row[9])] 

                first = False


    return info

def plot_comp_time(avg_comp_time, times, results_dir):
    
    # Bar plot 1
    f = plt.figure(figsize=(22, 12))
    f.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.bar(times, avg_comp_time, width=0.008)
    # myFmt = matplotlib.dates.DateFormatter('%H:%M')
    # ax = plt.axes()
    # ax.xaxis.set_major_formatter(myFmt)
    plt.xlabel('Repeat time (hh:mm)', fontsize=22, weight='bold') 
    plt.ylabel('Mean computation time (ms)', fontsize=22, weight='bold')
    plt.xticks(fontsize=20) 
    plt.yticks(fontsize=20) 
    plt.ylim([50, 150])
    plt.title('Mean localization computation time for each run', fontsize=22, weight='bold')

    plt.savefig('{}/avg_comp_time.png'.format(results_dir), format='png')
    plt.close()

    # Bar plot 2
    # plt.figure(figsize=(20, 12))
    # times_hm = []
    # for i in range(len(times)):
    #     times_hm.append(times[i].strftime('%H:%M'))
    # plt.bar(times_hm, avg_comp_time)
    # plt.ylabel('Avg. computation time (ms)', fontsize=20, weight='bold')
    # plt.xticks(np.arange(len(times_hm)), times_hm, fontsize=16, rotation=-30) 
    # plt.yticks(fontsize=16) 
    # plt.ylim([80, 150])
    # plt.title('Avg. localization computation time for each run', fontsize=20, weight='bold')

    # plt.savefig('{}/avg_comp_time_bar2.png'.format(results_dir), format='png')
    # plt.close()


    # # Line plot
    # plt.figure(figsize=(20, 12))
    # plt.plot(times, avg_comp_time, 'o-', linewidth=3)
    # myFmt = matplotlib.dates.DateFormatter('%H:%M')
    # ax = plt.axes()
    # ax.xaxis.set_major_formatter(myFmt)
    # plt.ylabel('Avg. computation time (ms)', fontsize=20, weight='bold')
    # plt.xticks(fontsize=16) 
    # plt.yticks(fontsize=16) 
    # plt.ylim([80, 150])
    # plt.title('Avg. localization computation time for each run', fontsize=20, weight='bold')

    # plt.savefig('{}/avg_comp_time_line.png'.format(results_dir), format='png')
    # plt.close()

def plot_inliers(avg_inliers, times, inliers, results_dir):

    # Plot average number of inliers for each run
    f = plt.figure(figsize=(22, 12))
    f.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.bar(times, avg_inliers, width=0.008)
    # myFmt = matplotlib.dates.DateFormatter('%H:%M')
    # ax = plt.axes()
    # ax.xaxis.set_major_formatter(myFmt)
    plt.xlabel('Repeat time (hh:mm)', fontsize=22, weight='bold') 
    plt.ylabel('Mean number of inliers', fontsize=22, weight='bold')
    plt.xticks(fontsize=20) 
    plt.yticks(fontsize=20) 
    plt.ylim([100, 400])
    plt.title('Mean number of inliers for each run', fontsize=22, weight='bold')

    plt.savefig('{}/avg_inliers.png'.format(results_dir), format='png')
    plt.close()

    # Plot cumulative distribution of inliers for each run
    plt.figure(figsize=(20, 12)) #
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
        cumulative = np.cumsum(unity_values)
        p = plt.plot(base[:-1], cumulative, linewidth=3)
        plot_lines.append(p[0])
        labels.append(times[i])
        # labels.append(times[i].strftime('%H:%M'))

    plt.axvline(x=20.0, color='red', linewidth='3', linestyle='--')

    plt.legend(plot_lines, labels, prop={'size': 16})
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

        print(i)

        if i != 10:

            inliers.append(info[i]["inliers_rgb"])
            avg_inliers.append(sum(info[i]["inliers_rgb"]) / float(len(info[i]["inliers_rgb"])))
            avg_comp_time.append(sum(info[i]["comp_time"]) / float(len(info[i]["comp_time"])))

            dt = datetime.datetime.fromtimestamp(info[i]["timestamp"][0] / 1e9)	
            # times.append(dt.strftime('%H:%M'))
            times.append(i)




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

    info = load_data(args.path, args.numrepeats)

    plot_data(info, args.path);
