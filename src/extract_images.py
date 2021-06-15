#! /usr/bin/env python2

import rospy
import rosbag
import sys
import rospkg
import numpy as np

def main():
    args = ["rosbag", "path"]
    if len(sys.argv) != len(args) + 1:
        print("Usage: extract_images_bag.py " + " ".join(args))
        return
    bag = rosbag.Bag(sys.argv[1], "r")
    img_path = sys.argv[2]
    im_topic = "/cam_fr/image_raw/compressed"

    for topic, msg, bagstamp in bag.read_messages(im_topic):
        stamp = msg.header.stamp
        if "jpeg" in msg.format:
            fmt = "jpeg"
        elif "png" in msg.format:
            fmt = "png"
        else:
            fmt = "raw"
        with open(img_path + "/{}.{}".format(stamp, fmt), "wb+") as f:
            f.write(msg.data)

if __name__ == "__main__":
    main()