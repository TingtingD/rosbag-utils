import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import laspy
import os
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
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
        PointField('color_r', 16, PointField.UINT8, 1),
        PointField('color_g', 17, PointField.UINT8, 1),
        PointField('color_b', 18, PointField.UINT8, 1)
    ]
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1)
    ]
    times = []
    with open(path + "/timestamps.txt", 'r') as f:
        while 1:
            time = f.readline()
            if time == "":
                f.close()
                break    
            times.append(float(time))
    cnt_lasfiles = len(file_paths)-1
    with rosbag.Bag(pathOut + "/" + bagName + ".bag", 'w') as bag:
        if len(times) != cnt_lasfiles: 
            raise Exception("Number of Time Stamps is Not Equal to Number of Las Files")
        for idx in tqdm(range(cnt_lasfiles)):
            file_path = file_root + "/" + str(idx) + ".las"
            if rospy.is_shutdown():
                break
            if file_path in file_paths: 
                lasData = laspy.read(file_path)
                if len(lasData.red)>0 and len(lasData.green)>0 and len(lasData.blue)>0: 
                    has_rgb = True
                else: 
                    has_rgb = False
                x_length = len(lasData.x)
                cloud_points = []
                for i in range(x_length):
                    if has_rgb:
                        cloud_points.append([lasData.x[i], lasData.y[i], lasData.z[i], lasData.intensity[i], lasData.red[i], lasData.green[i], lasData.blue[i]])
                    else:
                        cloud_points.append([lasData.x[i], lasData.y[i], lasData.z[i], lasData.intensity[i]])

                header = Header()
                if has_rgb:
                    cloud_msg = pc2.create_cloud(header, fields_with_rgb, cloud_points)
                else:
                    cloud_msg = pc2.create_cloud(header, fields, cloud_points)
                timestamp = rospy.Time.from_sec(lasData.gps_time[0]/1e9)
                cloud_msg.header.stamp = timestamp
                cloud_msg.header.seq = idx 
                cloud_msg.header.frame_id = frame_id
                
                bag.write(topicName, cloud_msg, timestamp)
