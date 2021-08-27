import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import datetime
import argparse
from PIL import Image, ImageDraw, ImageFont

def load_timstamp(data_dir, run_ind):

    results_dir = "{}/graph.index/repeats/{}/results".format(data_dir, run_ind)
    info_file_path = "{}/info.csv".format(results_dir) 

    with open(info_file_path) as csv_file:

        csv_reader = csv.reader(csv_file, delimiter=',')
        row_ind = 0

        for row in csv_reader:

            if row_ind == 1:
                timestamp = int(row[0]) 

            row_ind+=1

    return timestamp

def draw_label(images_path, video_path, run_id, i, description_str, dt, image_index):

    image_file = "{}/{}.png".format(images_path, i)
    image_file_new = "{}/{}.png".format(video_path, str(image_index).zfill(6))

    dm_str  = dt.strftime('%d/%m')
    hm_str  = dt.strftime('%H:%M')

    img = Image.open(image_file)
    draw_img = ImageDraw.Draw(img)

    fnt = ImageFont.truetype("Pillow/Tests/fonts/FreeMono.ttf", 20)
    draw_img.multiline_text((5,10), 
                             "{}\n{}\n{}".format(description_str, dm_str, hm_str), 
                             font=fnt, 
                             fill=(0, 255, 0), 
                             stroke_width=2, 
                             stroke_fill="black")
    
    img.save(image_file_new, "png")
    img.close()
    del draw_img


def label_images(timestamp, data_dir, image_index, run_id, start, end, step=1):  

    results_dir = "{}/graph.index/repeats/{}/results".format(data_dir, run_id)
    images_path = "{}/images".format(results_dir)
    video_path = "{}/graph.index/repeats/video".format(data_dir)

    if run_id == 0:
        description_str = "Teach"
    else:
        description_str = "Repeat"

    dt = datetime.datetime.fromtimestamp(timestamp / 1e9) 
   
    i = start 
    while i <= end + 1:

        draw_label(images_path, video_path, run_id, i, 
                   description_str, dt, image_index)

        if i != (end + 1):
            image_index +=1

        if (run_id == 7) and (i > 1265):
            step = 2

        i += step

    draw_label(images_path, video_path, run_id, end+1, description_str, dt, image_index)
    image_index += 1

    return image_index    


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=None, type=str,
                        help='path to results dir (default: None)')

    args = parser.parse_args()

    run_info = {1:[0,507], 2:[467,847], 3:[831,1118], 4:[1178,1516],
                7:[1048,1563], 8:[1115,1432], 11:[3136,3721], 12:[3838,4115],
                13:[2111,2408], 19:[3291,3816], 20:[1190,1292], 21:[1354,1533],
                22:[1539,1701], 24:[1616,1858], 25:[1811,1946], 26:[1948,2070],
                27:[2118,2266], 28:[2162,2222], 29:[2691,2818]} 
    
    image_index = 0
    for run in run_info.keys():

        timestamp = load_timstamp(args.path, run)

        # dt = datetime.datetime.fromtimestamp(timestamp / 1e9) 
        # print("{}-{}".format(run, dt.strftime('%H:%M')))

        step = 1 if run in [7,21, 22] else 2
   
        image_index = label_images(timestamp, args.path, image_index, run, 
                                   run_info[run][0], run_info[run][1], step);


