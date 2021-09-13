import numpy as np
from PIL import Image, ImageDraw, ImageFont
import argparse
import glob
import re
import os

def draw_times(img_dir):

    file_names = glob.glob("{}/*.png".format(img_dir))

    for fn in file_names:

        numbers = re.findall('\d+', fn)

        fn_stem = os.path.splitext(fn)[0]
        image_file_new = "{}{}.png".format(fn_stem, '_ts')
        
        img_base = Image.open(fn).convert("RGBA")
        
        img = Image.new("RGBA", img_base.size, (255,255,255,0))
        draw_img = ImageDraw.Draw(img)
        x, y = 305,305
        text = "{}:{}".format(numbers[-2], numbers[-1])
        fnt = ImageFont.truetype("Pillow/Tests/fonts/FreeMono.ttf", 60)
        w, h = fnt.getsize(text)
        draw_img.rectangle((x, y, x + w + 10, y + h + 10), fill=(0, 0, 0, 150))
        draw_img.multiline_text((x,y), 
                                 text, 
                                 font=fnt, 
                                 fill=(255, 255, 255, 150), 
                                 stroke_width=2, 
                                 stroke_fill=(255, 255, 255, 150))

        out = Image.alpha_composite(img_base, img)

        out.save(image_file_new, "png")
        out.close()
        img.close()
        img_base.close()
        del draw_img

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=None, type=str,
                        help='path to results dir (default: None)')

    args = parser.parse_args()

    draw_times(args.path)