#!/usr/bin/python

"""

Description: 
    1. Publishes camera_info topics with the same header as corresponding images
To Run: Example1:  python camera_info_publisher.py --left=left.yaml --right=right.yaml

"""

from __future__ import print_function

import rospy
import cv2
from sensor_msgs.msg import CameraInfo
import numpy as np
#from cv_bridge import CvBridge, CvBridgeError

#import message_filters
import argparse
import yaml
from rosbag import Bag
from tqdm import tqdm


def parse_yaml_file(caminfo_filename):
    with open(caminfo_filename, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = calib_data["camera_name"]
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]

    return camera_info_msg

def main():
    parser = argparse.ArgumentParser(description='ROS topics modifier')
    parser.add_argument("--left", required=True, help="Path to left camera info yaml file")
    parser.add_argument("--right", required=True, help="Path to right camera info yaml file")
    parser.add_argument("input", help="Path to input bag file")
    parser.add_argument("output", help="Path to output bag file")
    args = parser.parse_args()

    left_msg = parse_yaml_file(args.left)
    right_msg = parse_yaml_file(args.right)

    with Bag(args.input) as ibag, Bag(args.output, "w") as obag:
        for topic, msg, t in tqdm(ibag.read_messages()):
            if topic == "/cam_fl/camera_info":
                left_msg.header.stamp = msg.header.stamp
                obag.write(topic, left_msg, t)
            elif topic == "/cam_fr/camera_info":
                right_msg.header.stamp = msg.header.stamp
                obag.write(topic, right_msg, t)
            else:
                obag.write(topic, msg, t)

if __name__ == "__main__":
    main()
