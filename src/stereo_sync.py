#! /usr/bin/env python2

from __future__ import print_function

import rospy
import rosbag
import message_filters
import argparse
from tqdm import tqdm
from collections import OrderedDict

def sync(left_msg, right_msg, left_info, right_info):
  # Matches timestamps in left and right messages
  avg_stamp = rospy.Time(nsecs=(left_msg.header.stamp.to_nsec() + right_msg.header.stamp.to_nsec()) / 2)

  if left_msg.header.stamp != left_info.header.stamp:
    print("WARN Mismatched left camera info. Image: %s, Camera Info: %s, Difference: %s"
          % (left_msg.header.stamp, left_info.header.stamp,
             abs(left_msg.header.stamp.to_sec() - left_info.header.stamp.to_sec())))
  if right_msg.header.stamp != right_info.header.stamp:
    print("WARN Mismatched right camera info. Image: %s, Camera Info: %s, Difference: %s"
          % (right_msg.header.stamp, right_info.header.stamp,
             abs(right_msg.header.stamp.to_sec() - right_info.header.stamp.to_sec())))

  left_msg.header.stamp = avg_stamp
  right_msg.header.stamp = avg_stamp
  left_info.header.stamp = avg_stamp
  right_info.header.stamp = avg_stamp

  return left_msg, right_msg, left_info, right_info

def sync_bag(left_msg, right_msg, left_info, right_info, bag, topics, bag_timestamps):
  new_msgs = sync(left_msg, right_msg, left_info, right_info)
  for topic, msg in zip(topics, new_msgs):
    t = bag_timestamps.get(msg, None)
    assert t is not None, "Message out of cache at time %s (%s)" % (msg.header.stamp, topic)
    bag.write(topic, msg, t)

def sync_topic(left_msg, right_msg, left_info, right_info, publishers):
  new_msgs = sync(left_msg, right_msg, left_info, right_info)
  for msg, pub in zip(new_msgs, publishers):
    pub.publish(msg)

def topic_names(left, right, raw):
  image_suffix = "/image_raw" + ("" if raw else "/compressed")
  info_suffix = "/camera_info"

  return [left + image_suffix, right + image_suffix, left + info_suffix, right + info_suffix]

def bag_main(args):
  topics = topic_names(args.left, args.right, args.raw)

  filters = [message_filters.SimpleFilter() for _ in topics]
  sync_filter = message_filters.ApproximateTimeSynchronizer(filters, args.cache_size, args.slop)

  bag_timestamps = OrderedDict()  # Dictionary of messages to bag timestamps (max size: 4 * cache_size)

  with rosbag.Bag(args.ibag, 'r') as ibag, rosbag.Bag(args.obag, 'w') as obag:
    sync_filter.registerCallback(sync_bag, obag, topics, bag_timestamps)
    for topic, msg, t in tqdm(ibag.read_messages(), total=ibag.get_message_count()):
      if topic in topics:
        bag_timestamps[msg] = t
        if len(bag_timestamps) > len(topics) * args.cache_size:
          bag_timestamps.popitem(last=False)
        filters[topics.index(topic)].signalMessage(msg)
      else:
        obag.write(topic, msg, t)

def node_main(args):
  from sensor_msgs.msg import Image, CompressedImage, CameraInfo
  rospy.init_node("stereo_sync")

  topics = topic_names(args.left, args.right, args.raw)
  new_topics = topic_names(args.left + "_sync", args.right + "_sync", args.raw)
  message_types = [Image if args.raw else CompressedImage] * 2 + [CameraInfo] * 2
  
  filters = [message_filters.Subscriber(topic, message_type) for topic, message_type in zip(topics, message_types)]
  publishers = [rospy.Publisher(topic, message_type, queue_size=100) for topic, message_type in zip(new_topics, message_types)]
  sync_filter = message_filters.ApproximateTimeSynchronizer(filters, args.cache_size, args.slop)
  sync_filter.registerCallback(sync_topic, publishers)
  rospy.spin()

def main():
  argp = argparse.ArgumentParser("Uses ApproximateTimeSynchronizer to force synchronized stereo images")
  argp.add_argument("ibag", nargs="?", metavar="IN", help="Path to input rosbag file (run live node if empty)")
  argp.add_argument("obag",  nargs="?", metavar="OUT", help="Path to output rosbag file (run live node if empty)")
  argp.add_argument("--left", default="/cam_fl", help="Left topic")
  argp.add_argument("--right", default="/cam_fr", help="Right topic")
  argp.add_argument("--raw", action="store_true", help="Do not use compressed topics")
  argp.add_argument("--cache_size", default=100, type=int, help="Cache size")
  argp.add_argument("--slop", default=0.02, type=float, help="Maximum deviation between messages")
  args = argp.parse_args(rospy.myargv()[1:])

  if args.ibag is None and args.obag is None:
    node_main(args)
  elif args.ibag is None or args.obag is None:
    argp.error("Both IN and OUT must be specified to process rosbags")
  else:
    bag_main(args)

if __name__ == "__main__":
  main()
