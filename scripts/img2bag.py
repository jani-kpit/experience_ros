#!/usr/bin/env python

# run as 'rosrun experience_ros img2bag.py <image_folder> <bagfile>'

import sys, os
from ros import rosbag
import rospy
from sensor_msgs.msg import Image
import time

from PIL import ImageFile

def GetImages(dir):
    print("Scanning directory %s for png and jpg" % dir)
    imgs = []
    if os.path.exists(dir):
        for f in os.listdir(dir):
            ext = os.path.splitext(f)[1]
            if ext in ['.png', '.jpg']:
                imgs.append(os.path.join(dir, f))
    return imgs

def CreateBag(args):
    imgs = GetImages(args[0])
    if not imgs:
        print("No images found in %s" % dir)
        exit()

    rosbagfile = args[1]
    if (os.path.exists(rosbagfile)):
        os.remove(rosbagfile)

    bag = rosbag.Bag(rosbagfile, 'w')

    try:
        for filename in imgs:
            print("Adding %s" % filename)
            with open(filename, "rb") as image_data:
                parser = ImageFile.Parser()
                while 1:
                    raw_bytes = image_data.read(1024)
                    if not raw_bytes:
                        break
                    parser.feed(raw_bytes)

                parsed_image = parser.close()

                timestamp = rospy.Time.from_sec(time.time())

                image = Image()
                image.header.stamp = timestamp
                image.width = parsed_image.size[0]
                image.height = parsed_image.size[1]
                image.step = image.width * 4;
                image.encoding = "rgba8"
                image.header.frame_id = "image_data/image"
                image.data = [pix for pixdata in parsed_image.getdata() for pix in pixdata]

                bag.write('image_node/image_raw', image, timestamp)
    finally:
        bag.close()

if __name__ == "__main__":
    if len(sys.argv) == 3:
        CreateBag(sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir bagfile")