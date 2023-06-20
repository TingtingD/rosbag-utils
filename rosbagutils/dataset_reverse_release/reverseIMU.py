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



# /data/usr/bin/python3.7 /data/home/airlab/Documents/rosbag-utils/rosbagutils/dataset_reverse_release/reverseIMU.py
#
#reverseIMU("/data/home/airlab/run_7/bags/dataset_release_2023-05-26_18-20-46_3a8e/cmu_sp1_imu_data/imu_data.csv", "Dang", "/data/home/airlab/run_7/bags")



'''
def processIMU(paths, targetTopic, pathOut, sendProgress):
    with open(pathOut + "/imu_data.csv", "w") as f:
        f.write("timestamp, q_x, q_y, q_z, q_w, ang_vel_x, ang_vel_y, ang_vel_z, lin_acc_x, lin_acc_y, lin_acc_z\n")
        for path, pathIdx in zip(paths, range(len(paths))):
            if path.strip() == "":
                continue
            print("Processing " + path)

            bagIn = rosbag.Bag(path)
            
            topicsInfo = bagIn.get_type_and_topic_info().topics
            
            for topic, msg, t in bagIn.read_messages(topics=[targetTopic]):
                timestamp = str(t)
                q_x, q_y, q_z, q_w = (
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                )

                ang_vel_x, ang_vel_y, ang_vel_z = (
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                )

                lin_acc_x, lin_acc_y, lin_acc_z = (
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                )

                f.write(
                    timestamp
                    + ","
                    + str(q_x)
                    + ","
                    + str(q_y)
                    + ","
                    + str(q_z)
                    + ","
                    + str(q_w)
                    + ","
                    + str(ang_vel_x)
                    + ","
                    + str(ang_vel_y)
                    + ","
                    + str(ang_vel_z)
                    + ","
                    + str(lin_acc_x)
                    + ","
                    + str(lin_acc_y)
                    + ","
                    + str(lin_acc_z)
                    + "\n"
                )


'''