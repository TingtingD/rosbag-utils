import json
import os
import argparse
import rosbag
import rospy
from reverseIMU import reverseIMU
from reversePointcloud import reversePointcloud


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Script for reversing result files from rosbagutils API.\
            \nOrganize the result files following the instructions below.\
            \nGive the path to this folder in the --datapath argument."
    )