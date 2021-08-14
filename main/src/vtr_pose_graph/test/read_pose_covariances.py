import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import datetime
import argparse

def load_covariances(data_dir, num_repeats):

    cov = []

    for i in range(0, num_repeats):

        cov.append({"xx":[],
                    "yy":[],
                    "zz":[],
                    "rr":[],
                    "pp":[],
                    "ww":[]})

        results_dir = "{}/graph.index/repeats/{}/results".format(data_dir, i+1)
        info_file_path = "{}/covariances.csv".format(results_dir) 

        with open(info_file_path) as csv_file:

            csv_reader = csv.reader(csv_file, delimiter=',')
            first = True

            for row in csv_reader:

                if not first:
                    cov[i]["xx"] += [float(row[4])]
                    cov[i]["yy"] += [float(row[11])]
                    cov[i]["zz"] += [float(row[18])]
                    cov[i]["rr"] += [float(row[25])]
                    cov[i]["pp"] += [float(row[32])]
                    cov[i]["ww"] += [float(row[39])]
                    
                first = False


    return cov


def plot_cov(cov, results_dir):

    # Plot average number of inliers for each run
    f = plt.figure(figsize=(30, 12))
    f.tight_layout(rect=[0, 0.03, 1, 0.95])

    plot_lines = []
    labels = []
    
    for key in cov[0].keys():
        if key in ['xx', 'yy', 'zz']:
            p = plt.plot(cov[0][key])
            plot_lines.append(p[0])
            labels.append(key)
   
    plt.ylim([0, 0.0003])
    plt.xlabel('Vertex', fontsize=22, weight='bold') 
    plt.ylabel('Cov', fontsize=22, weight='bold')
    plt.xticks(fontsize=20) 
    plt.yticks(fontsize=20) 
    plt.title('Covariances', fontsize=22, weight='bold')
    plt.legend(plot_lines, labels, prop={'size': 16})

    plt.savefig('{}/cov_trans.png'.format(results_dir), bbox_inches='tight', format='png')
    plt.close()

    f = plt.figure(figsize=(30, 12))
    f.tight_layout(rect=[0, 0.03, 1, 0.95])

    plot_lines = []
    labels = []
    
    for key in cov[0].keys():
        if key in ['rr', 'pp', 'ww']:
            p = plt.plot(cov[0][key])
            plot_lines.append(p[0])
            labels.append(key)
   
    plt.ylim([0, 0.00002])
    plt.xlabel('Vertex', fontsize=22, weight='bold') 
    plt.ylabel('Cov', fontsize=22, weight='bold')
    plt.xticks(fontsize=20) 
    plt.yticks(fontsize=20) 
    plt.title('Covariances', fontsize=22, weight='bold')
    plt.legend(plot_lines, labels, prop={'size': 16})

    plt.savefig('{}/cov_rot.png'.format(results_dir), bbox_inches='tight', format='png')
    plt.close()

def plot_data(cov, data_dir, bad_gps):
    
    results_dir = "{}/graph.index/repeats".format(data_dir)

    plot_cov(cov, results_dir)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=None, type=str,
                        help='path to results dir (default: None)')
    parser.add_argument('--numrepeats', default=None, type=int,
                        help='number of repeats (default: None)')

    args = parser.parse_args()

    cov = load_covariances(args.path, args.numrepeats)

    bad_gps = []

    plot_data(cov, args.path, bad_gps);
