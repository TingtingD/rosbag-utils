import rosbag
import rospy
import json
import yaml
import subprocess
import traceback
import numpy as np
import math
import pandas as pd 
from sensor_msgs.msg import Imu, NavSatFix
from tqdm import tqdm 
import os

'''
reverseIMU: csv file with imu data format => .bag file 
path: input path to the csv file that contains imu data 
bagName: the name of the output bag file 
pathOut: output path to the bag file 
topicName: name of the topic in the bag file, starting with "/"
'''
def reverseIMU(path, topicName, frame_id=None):
    for root, directories, files in os.walk(path):
        for file in files:
            path = os.path.join(root, file)
    df = pd.read_csv(path)

    if topicName[0]!='/': 
        topicName = "/" + topicName

    first_flag = True
    pub = rospy.Publisher(topicName, Imu, queue_size=100)
    rospy.init_node(topicName[1:] + "Node")

    for row in tqdm(range(df.shape[0])):
        if rospy.is_shutdown():
            break

        line = str(df['timestamp'][row])
        sec = int(line[:-9])
        nsec = int(line[-9:])
        timestamp = rospy.Time(sec, nsec)

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
        if frame_id is not None: 
            imu_msg.header.frame_id = frame_id

        if first_flag:
            first_flag = False
            prev_time = timestamp
        else:
            rospy.sleep((timestamp-prev_time).to_sec())
            #print(timestamp, " ", prev_time, " ", timestamp-prev_time, " ", (timestamp-prev_time).to_sec())
            prev_time = timestamp

        pub.publish(imu_msg)

        #bag.write(topicName, imu_msg, timestamp)

        # gps_msg = NavSatFix()
        # gps_msg.header.stamp = timestamp

        # Populate the data elements for GPS

        # bag.write("/gps", gps_msg, timestamp)

if __name__ == '__main__':
    folder = "/data/home/airlab/Documents/Sample_dataset/results/imu"
    reverseIMU(folder, "/imu", None)
