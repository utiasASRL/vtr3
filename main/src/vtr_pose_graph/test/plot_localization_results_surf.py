import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.dates import date2num
import datetime
import argparse
import tikzplotlib

def load_data(data_dir_learned, data_dir_surf,  num_repeats, ignore_runs_learned, ignore_runs_surf):

    info_learned = []
    info_surf = []
   
    for i in range(0, num_repeats):

        if (i+1) in ignore_runs_learned:
            continue

        info_learned.append({"timestamp":[],
                             "live_id":[],
                             "priv_id":[],
                             "success":[],
                             "inliers_rgb":[],
                             "inliers_gray":[],
                             "inliers_cc":[],
                             "window_temporal_depth":[],
                             "window_num_vertices":[],
                             "comp_time":[]})

        results_dir_learned = "{}/graph.index/repeats/{}/results".format(data_dir_learned, i+1)
        info_file_path_learned = "{}/info.csv".format(results_dir_learned) 
        
        with open(info_file_path_learned) as csv_file:

            csv_reader = csv.reader(csv_file, delimiter=',')
            first = True

            for row in csv_reader:

                if not first:
                    info_learned[-1]["timestamp"] += [int(row[0])]
                    info_learned[-1]["live_id"] += [row[1]]
                    info_learned[-1]["priv_id"] += [row[2]]
                    info_learned[-1]["success"] += [row[3]]
                    info_learned[-1]["inliers_rgb"] += [float(row[4])]
                    info_learned[-1]["inliers_gray"] += [float(row[5])]
                    info_learned[-1]["inliers_cc"] += [float(row[6])]
                    info_learned[-1]["window_temporal_depth"] += [row[7]]
                    info_learned[-1]["window_num_vertices"] += [row[8]]
                    info_learned[-1]["comp_time"] += [float(row[9])]

                first = False

        info_surf.append({"timestamp":[],
                              "live_id":[],
                              "priv_id":[],
                              "success":[],
                              "inliers_rgb":[],
                              "inliers_gray":[],
                              "inliers_cc":[],
                              "window_temporal_depth":[],
                              "window_num_vertices":[],
                              "comp_time":[]})

        if not (i + 1) in ignore_runs_surf:

            results_dir_surf = "{}/graph.index/repeats/{}/results".format(data_dir_surf, i+1)
            info_file_path_surf = "{}/info.csv".format(results_dir_surf) 

            with open(info_file_path_surf) as csv_file:

                csv_reader = csv.reader(csv_file, delimiter=',')
                first = True

                for row in csv_reader:

                    if not first:
                        info_surf[-1]["timestamp"] += [int(row[0])]
                        info_surf[-1]["live_id"] += [row[1]]
                        info_surf[-1]["priv_id"] += [row[2]]
                        info_surf[-1]["success"] += [row[3]]
                        info_surf[-1]["inliers_rgb"] += [float(row[4])]
                        info_surf[-1]["inliers_gray"] += [float(row[5])]
                        info_surf[-1]["inliers_cc"] += [float(row[6])]
                        info_surf[-1]["window_temporal_depth"] += [row[7]]
                        info_surf[-1]["window_num_vertices"] += [row[8]]
                        info_surf[-1]["comp_time"] += [float(row[9])]

                    first = False
        else:
            info_surf[-1]["timestamp"] = info_learned[-1]["timestamp"]

        dt = datetime.datetime.fromtimestamp(info_learned[-1]["timestamp"][0] / 1e9) 
        print("{}-{}".format(i+1, dt.strftime('%H:%M')))

    return info_learned, info_surf

def plot_inliers(avg_inliers_learned, avg_inliers_surf, times_learned, times_surf, colours_learned, colours_surf, results_dir):

    plt.rcParams['text.latex.preamble']=[r"\usepackage{lmodern}"]
    params = {'text.usetex' : True,
              'font.size' : 40,                   # Set font size to 11pt
              'axes.labelsize': 40,               # -> axis labels
              'legend.fontsize': 40,              # -> legends
              'xtick.labelsize' : 40,
              'ytick.labelsize' : 40,
              'font.family' : 'lmodern',
              'text.latex.unicode': True,
              }
    plt.rcParams.update(params) 

    x = date2num(times_learned)

    # Plot average number of inliers for each run
    f = plt.figure(figsize=(50, 10))
    # f = plt.figure(figsize=(40, 18))

    f.tight_layout(rect=[0, 0.03, 1, 0.95])

    min_surf = 1000
    
    for k in range(len(avg_inliers_learned)):
        if avg_inliers_surf[k] == 0:
            p1 = plt.bar(x[k], avg_inliers_learned[k], width=0.008) # 0.015 #0.001
        else:
            p1 = plt.bar(x[k] - 0.004, avg_inliers_learned[k], width=0.008) # 0.015 #0.001
    
        p1[0].set_color(colours_learned['day'][k])

        if avg_inliers_surf[k] != 0:
            p2 = plt.bar(x[k] + 0.004, avg_inliers_surf[k], width=0.008) # 0.015 #0.001

            if avg_inliers_surf[k] < min_surf:
                min_surf = avg_inliers_surf[k]
    
        p2[0].set_color(colours_surf['day'][k])
    
   
    myFmt = matplotlib.dates.DateFormatter('%H:%M')
    ax = plt.axes()
    ax.xaxis.set_major_formatter(myFmt)

    plt.xlim([min(times_learned) - datetime.timedelta(minutes=10), max(times_learned) + datetime.timedelta(minutes=10)])
    plt.xlabel(r'\textbf{Repeat time (hh:mm)}', fontsize=50) 
    plt.ylabel(r'\textbf{Mean number of inliers}', fontsize=50)
    plt.xticks(fontsize=48) 
    plt.yticks(fontsize=48) 
    # plt.ylim([100, 450])
    # plt.ylim([40, 350])
    plt.ylim([max(min_surf - 10, 0), max(avg_inliers_learned) + 10])
    
    # plt.title('Mean number of inliers for each repeat', fontsize=26, weight='bold')

    # legend_elements = [matplotlib.lines.Line2D([0], [0], color='C0', lw=4, label='Day1: 03.08'),
    #                    matplotlib.lines.Line2D([0], [0], color='C1', lw=4, label='Day2: 09.08')]
    legend_elements = [matplotlib.lines.Line2D([0], [0], color='C0', lw=4, label='Day1: 15/08'),
                       matplotlib.lines.Line2D([0], [0], color='C1', lw=4, label='Day2: 16/08'),
                       matplotlib.lines.Line2D([0], [0], color='C2', lw=4, label='Day3: 20/08'),
                       matplotlib.lines.Line2D([0], [0], color='gray', lw=4, label='SURF')]                   
    plt.legend(handles=legend_elements, fontsize=48);

    plt.savefig('{}/avg_inliers_learned_surf.pdf'.format(results_dir), dpi=1000, bbox_inches='tight', format='pdf')
    # tikzplotlib.save('{}/avg_inliers_learned_surf.tex'.format(results_dir))
    plt.close()

def plot_data(info_learned, info_surf, data_dir_learned, data_dir_surf):

    avg_inliers_learned = []
    avg_inliers_surf = []
    times_learned = []
    times_surf = [] 
    colours_learned = {'day':[], 'gps':[]}
    colours_surf = {'day':[], 'gps':[]}

    for i in range(len(info_surf)):

        if info_surf[i]["inliers_gray"] == []:
            avg_inliers_surf.append(0)
        else:
            avg_inliers_surf.append((sum(info_surf[i]["inliers_gray"]) + sum(info_surf[i]["inliers_cc"])) / float(len(info_surf[i]["inliers_gray"])))
        
        dt = datetime.datetime.fromtimestamp(info_surf[i]["timestamp"][0] / 1e9)

        if dt.day == 16:
            colours_surf['day'] = colours_surf['day'] + ['gray']
        elif dt.day == 15:
            colours_surf['day'] = colours_surf['day'] + ['gray']
        else:
            colours_surf['day'] = colours_surf['day'] + ['gray']

        # Cheat to get all the plot bars for one day
        if dt.day != 15:
            dt = dt.replace(day=15)
       
        times_surf.append(dt)

    for i in range(len(info_learned)):

        avg_inliers_learned.append(sum(info_learned[i]["inliers_rgb"]) / float(len(info_learned[i]["inliers_rgb"])))

        dt = datetime.datetime.fromtimestamp(info_learned[i]["timestamp"][0] / 1e9)	

        if dt.day == 16:
            colours_learned['day'] = colours_learned['day'] + ['C1']
        elif dt.day == 15:
            colours_learned['day'] = colours_learned['day'] + ['C0']
        else:
            colours_learned['day'] = colours_learned['day'] + ['C2']

        # Cheat to get all the plot bars for one day
        if dt.day != 15:
            dt = dt.replace(day=15)
       
        times_learned.append(dt)

    results_dir = "{}/graph.index/repeats".format(data_dir_learned)

    print(avg_inliers_learned)
    print(avg_inliers_surf)

    plot_inliers(avg_inliers_learned, avg_inliers_surf, times_learned, times_surf, colours_learned, colours_surf, results_dir)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=None, type=str,
                        help='path to results dir (default: None)')
    parser.add_argument('--paths', default=None, type=str,
                        help='path to results dir (default: None)')
    parser.add_argument('--numrepeats', default=None, type=int,
                        help='number of repeats (default: None)')

    args = parser.parse_args()

    # ignore_runs = [5,6,9,10,14, 15, 16, 17, 18, 23] # exp2 wrong
    ignore_runs_learned = [6,7,10,14,15,16,17,18,24] # exp2
    # ignore_runs = [1,2,3,4,6,31,35,39,40,10,16,21,19] #exp1 org
    # ignore_runs = []
    # ignore_runs = [4, 6,31,34,35,36,39,40,10,16,21,19] #exp1 
    ignore_runs_surf = [6,7,8,10,11,12,13,14,15,16,17,18,20,21,22,24,29] # exp2 surf

   
    info_learned, info_surf = load_data(args.path, args.paths, args.numrepeats, ignore_runs_learned, ignore_runs_surf)

    plot_data(info_learned, info_surf, args.path, args.paths);
