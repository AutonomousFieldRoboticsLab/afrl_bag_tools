#! /usr/bin/env python2

import rosbag
import argparse

def main():
  argp = argparse.ArgumentParser("Plots xy path of odometries from rosbag")
  argp.add_argument("ibag", metavar="IN", help="Path to input rosbag file")
  argp.add_argument("obag", metavar="OUT", help="Path to output rosbag file")
  argp.add_argument("-r", "--remap", nargs=2, metavar=("FROM", "TO"), action="append", default=[], help="Remap topics")
  args = argp.parse_args()

  remap = {k: v for k, v in args.remap}
  with rosbag.Bag(args.ibag, 'r') as ibag, rosbag.Bag(args.obag, 'w') as obag:
    for topic, msg, t in ibag.read_messages():
      obag.write(remap.get(topic, topic), msg, t)

if __name__ == "__main__":
  main()
