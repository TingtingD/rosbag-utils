import rosbag
import rospy 
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np
import laspy
import time
import os
from .. import utils
import random
from tqdm import tqdm 

'''
path: input path to a folder that contains a bunch of .las file and one timestamp file
path: input path to the csv file that contains imu data 
bagName: the name of the output bag file 
pathOut: output path to the bag file 
topicName: name of the topic in the bag file, starting with "/"
'''
def reversePointcloud(path, bagName, pathOut, topicName, frame_id):
    
    if topicName[0]!='/': 
        topicName = "/" + topicName
    
    folder_path = path
    file_paths = []
    file_root  = ""
    for root, directories, files in os.walk(folder_path):
        file_root = root
        for file in files:
            file_path = os.path.join(root, file)
            file_paths.append(file_path)
    
    # read from .las file and timestamp
    fields_with_rgb = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgba', 12, PointField.UINT32, 1),
    ]
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    times = []
    with open(path + "/timestamps.txt", 'r') as f:
        while 1:
            time = f.readline()
            if time == "": break
            times.append(float(time))
    cnt_lasfiles = len(file_paths)-1
    with rosbag.Bag(pathOut + "/" + bagName + ".bag", 'w') as bag:
        if len(times) != cnt_lasfiles: 
            raise Exception("Number of Time Stamps is Not Equal to Number of Las Files")
        for idx in tqdm(range(cnt_lasfiles)):
            file_path = file_root + "/" + str(idx) + ".las"
            if file_path in file_paths: 
                lasData = laspy.read(file_path)
                if len(lasData.red)>0 and len(lasData.green)>0 and len(lasData.blue)>0: 
                    has_rgb = True
                else: 
                    has_rgb = False
                x_length = len(lasData.x)
                if has_rgb:
                    cloud_points = np.zeros((x_length, 4), np.int32)
                else:
                    cloud_points = np.zeros((x_length, 3), np.int32)
                for i in range(x_length):
                    if has_rgb:
                        rgb_value = (lasData.red[i]<<16) + (lasData.green[i]<<8) + (lasData.blue[i])
                        cloud_points[i] = [lasData.x[i], lasData.y[i], lasData.z[i], rgb_value]
                    else:
                        cloud_points[i] = [lasData.x[i], lasData.y[i], lasData.z[i]]

                header = Header()
                #laspy.LasHeader(version="1.3", point_format=3)
                if has_rgb:
                    cloud_msg = pc2.create_cloud(header, fields_with_rgb, cloud_points.tolist())
                else:
                    cloud_msg = pc2.create_cloud(header, fields, cloud_points.tolist())
                #timestamp = rospy.Time.from_sec(times[idx]/1e9)
                timestamp = rospy.Time.from_sec(lasData.gps_time[0]/1e9)
                cloud_msg.header.stamp = timestamp
                cloud_msg.header.seq = idx 
                cloud_msg.header.frame_id = frame_id
                bag.write(topicName, cloud_msg, timestamp)

