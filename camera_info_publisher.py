#!/usr/bin/python

"""

Description: 
    1. Publishes camera_info topics with the same header as corresponding images
To Run: Example1:  python camera_info_publisher.py --left=left.yaml --right=right.yaml

"""

from __future__ import print_function

import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
#from cv_bridge import CvBridge, CvBridgeError

#import message_filters
import argparse
import yaml


class topic_modifier:

    def __init__(self):
        parser = argparse.ArgumentParser(description='ROS topics modifier')
        parser.add_argument("--left", help="Path to left camera info yaml file")
        parser.add_argument("--right", help="Path to right camera info yaml file")
        args = parser.parse_args()
        self.left_caminfo_filename = args.left
        self.right_caminfo_filename = args.right

        self.left_subscriber = rospy.Subscriber("/left/image_raw", Image, self.left_image_callback, queue_size=100)
        self.right_subscriber = rospy.Subscriber("/right/image_raw", Image, self.right_image_callback, queue_size=100)

        self.left_caminfo_pub = rospy.Publisher("/left/camera_info", CameraInfo, queue_size=100)
        self.right_caminfo_pub = rospy.Publisher("/right/camera_info", CameraInfo, queue_size=100)
       

    def parse_yaml_file(self, caminfo_filename, image_msg):

        with open(caminfo_filename, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = calib_data["camera_name"]
        camera_info_msg.header.stamp = image_msg.header.stamp
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]

        return camera_info_msg

    def left_image_callback(self, image_msg):
        # Publish camera_info
        left_camera_info_msg = self.parse_yaml_file(self.left_caminfo_filename, image_msg)
        self.left_caminfo_pub.publish(left_camera_info_msg)

    def right_image_callback(self, image_msg):
        # Publish camera_info
        right_camera_info_msg = self.parse_yaml_file(self.right_caminfo_filename, image_msg)  
        self.right_caminfo_pub.publish(right_camera_info_msg)


if __name__ == "__main__":
    
    topic_modifier = topic_modifier()
    
    rospy.init_node('topic_modifier', anonymous=True)
    while(not rospy.is_shutdown()):
        rospy.spin()
