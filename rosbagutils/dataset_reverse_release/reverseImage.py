import os
import rosbag
import rospy
import json
import random
import numpy as np
import math
import pandas as pd
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge
from tqdm import tqdm
import cv2

'''
reverseImage: 
path: a folder with .png images and a timestamps.txt file 
topicName: name of the topic to publish
'''


def reverseImage(path, bagName, pathOut, topicName, frame_id=None):
    file_paths = [os.path.join(path, x)
                  for x in os.listdir(path) if x.endswith('.png')]
    first_flag = True

    if topicName[0] != '/':
        topicName = "/" + topicName

    br = CvBridge()
    pub = rospy.Publisher(topicName, Image, queue_size=5)

    rospy.init_node("mynode")
    count = 0
    with open(path+'/timestamps.txt', 'r') as f:
        lines = f.readlines()
        assert (len(lines) == len(file_paths))
        for line, file_path in zip(lines, file_paths):
            if rospy.is_shutdown():
                break
            print("frame: ", count)

            line = line[:-1]
            sec = int(line[:-9])
            nsec = int(line[-9:])
            timestamp = rospy.Time(sec, nsec)

            if first_flag:
                first_flag = False
                prev_time = timestamp
            else:
                # print('time difference:', (timestamp-prev_time).to_sec())
                rospy.sleep((timestamp-prev_time).to_sec())
                prev_time = timestamp

            cv2image = cv2.imread(path+'/'+str(count)+'.png')
            count += 1
            image_msg = br.cv2_to_imgmsg(cv2image)
            image_msg.header.stamp = timestamp
            if frame_id is not None:
                image_msg.frame_id = frame_id

            pub.publish(image_msg)


if __name__ == '__main__':
    folder = "/data/home/tianhao/datasets/SubT/2021-05-25/bags/auto/1/cam_dropcam_drop"
    reverseImage(folder, None, None, "/uav_image_drop", None)
