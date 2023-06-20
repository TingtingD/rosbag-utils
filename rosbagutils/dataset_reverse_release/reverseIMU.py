import rosbag
import rospy
import json
import yaml
import subprocess
import traceback
from .. import utils
import random
import numpy as np
import math
import pandas as pd 
from sensor_msgs.msg import Imu, NavSatFix
from tqdm import tqdm 

'''
reverseIMU: csv file with imu data format => .bag file 
path: input path to the csv file that contains imu data 
bagName: the name of the output bag file 
pathOut: output path to the bag file 
topicName: name of the topic in the bag file, starting with "/"
'''
def reverseIMU(path, bagName, pathOut, topicName, frame_id):
    df = pd.read_csv(path)

    if topicName[0]!='/': 
        topicName = "/" + topicName

    with rosbag.Bag(pathOut + "/" + bagName + ".bag", 'w') as bag:
        for row in tqdm(range(df.shape[0])):
            timestamp = rospy.Time.from_sec(df['timestamp'][row]/1e9)
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp

            # Populate the data elements for IMU
            # e.g. imu_msg.angular_velocity.x = df['a_v_x'][row]
            
            imu_msg.orientation.x = df[" q_x"][row]
            imu_msg.orientation.y = df[" q_y"][row]
            imu_msg.orientation.z = df[" q_z"][row]
            imu_msg.orientation.w = df[" q_w"][row]

            imu_msg.angular_velocity.x = df[" ang_vel_x"][row]
            imu_msg.angular_velocity.y = df[" ang_vel_y"][row]
            imu_msg.angular_velocity.z = df[" ang_vel_z"][row]

            imu_msg.linear_acceleration.x = df[" lin_acc_x"][row]
            imu_msg.linear_acceleration.y = df[" lin_acc_y"][row]
            imu_msg.linear_acceleration.z = df[" lin_acc_z"][row]

            imu_msg.header.seq = row 
            imu_msg.header.frame_id = frame_id
            bag.write(topicName, imu_msg, timestamp)

            # gps_msg = NavSatFix()
            # gps_msg.header.stamp = timestamp

            # Populate the data elements for GPS

            # bag.write("/gps", gps_msg, timestamp)
