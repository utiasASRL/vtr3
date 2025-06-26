import os
import sys
import traceback
import argparse
import sqlite3
from io import BytesIO

def get_topic_list(bag_name):
    """ Return a list of topics stored in this bag """
    conn = sqlite3.connect(bag_name)
    cursor = conn.cursor()
    topics_data = cursor.execute("SELECT id, name, type FROM topics").fetchall()
    topic_info = {name_of:(id_of, type_of) for id_of,name_of,type_of in topics_data}
    raw_topic_id=topic_info['/novatel/oem7/oem7raw']
    print('ROS Topics Found in file:')
    for row in topic_info:
        print(row)
    if(raw_topic_id==0):
        print('\nNo Raw NovAtel Logs found in file.')
    return raw_topic_id[0]
def get_topic_messages(bag_name, topic, outpath):
    """
    Returns a list of messages for a specific topic in this bag
    """
    conn = sqlite3.connect(bag_name)
    cursor = conn.cursor()
    # Get from the db
    data = cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic)).fetchall()
    out_path = os.path.join(outpath, 'novatel_data.log')
    fout = open(out_path,'wb')
    for rows in data:
        fout.write(rows[1])
    return 0
def parse_args():
    usage = "Extract raw NovAtel data from a ROS2 Bag 'db3' file" \
            "  usage: ros2_to_raw -f <file.db3>"
    parser = argparse.ArgumentParser(usage=usage)
    p = argparse.ArgumentParser(description='')
    p.add_argument('-f', help='Absolute path to input file')
    return p.parse_args()

if __name__ == "__main__":
    # args = parse_args()
    cwd = os.getcwd()
    print(os.getcwd())
    bag_name = "/home/desiree/ASRL/vtr3/data/Testing/VirtualLidar/june17GPS/parking/rosbag2_2025_06_17-17_08_14_0.db3"
    out_path = "/home/desiree/ASRL/vtr3/data/Testing/VirtualLidar/june17GPS/parking/rosbag2_2025_06_17-17_08_14_0EXTRACTED"
    raw_topic_id = get_topic_list(bag_name)
    topic_msg = get_topic_messages(bag_name, topic=raw_topic_id,outpath=out_path)