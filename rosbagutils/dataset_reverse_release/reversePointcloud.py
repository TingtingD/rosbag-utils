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
'''
def reversePointcloud(path, bagName, pathOut, topicName):
    
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
    with rosbag.Bag(pathOut + "/" + bagName + ".bag", 'w') as bag:
        for idx in tqdm(range(len(file_paths))):
            file_path = file_root + "/" + str(idx) + ".las"
            #print(file_path)
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
                timestamp = rospy.Time.from_sec(lasData.gps_time[0]/1e9)
                bag.write(topicName, cloud_msg, timestamp)



def processPointcloud(paths, targetTopic, pathOut, sendProgress):
    def writeToFile(arrayX, arrayY, arrayZ, arrayT, arrayR, arrayG, arrayB):
        nonlocal outFileCount, totalNumPoints
        totalNumPoints += arrayX.size
        filename = pathOut + str(outFileCount) + ".las"
        header = laspy.LasHeader(version="1.3", point_format=3)
        lasData = laspy.LasData(header)
        lasData.x = arrayX.finalize()
        lasData.y = arrayY.finalize()
        lasData.z = arrayZ.finalize()
        lasData.gps_time = arrayT.finalize()
        if arrayR.size > 0 and arrayG.size > 0 and arrayB.size > 0:
            lasData.red = arrayR.finalize()
            lasData.green = arrayG.finalize()
            lasData.blue = arrayB.finalize()
        lasData.write(filename)
        outFileCount += 1

    def createArrs():
        return (
            utils.FastArr(),
            utils.FastArr(),
            utils.FastArr(),
            utils.FastArr(),
            utils.FastArr(),
            utils.FastArr(),
            utils.FastArr(),
        )

    utils.mkdir(utils.getFolderFromPath(pathOut))
    outFileCount = 0
    totalNumPoints = 0
    print("Exporting point cloud from " + targetTopic + " to " + pathOut)
    print("Input bags: " + str(paths))
    percentProgressPerBag = 1 / len(paths)

    arrayX, arrayY, arrayZ, arrayT, arrayR, arrayG, arrayB = createArrs()
    startTime = time.time_ns()
    totalArrayTime = 0
    count = -1
    with open(pathOut + "/timestamps.txt", "w") as f:
        for path, pathIdx in zip(paths, range(len(paths))):
            if path.strip() == "":
                continue
            print("Processing " + path)
            
            bagIn = rosbag.Bag(path)
            topicsInfo = bagIn.get_type_and_topic_info().topics
            
            bagStartCount = count
            for topic, msg, t in bagIn.read_messages(topics=[targetTopic]):
                count += 1
                
                arrayTimeStart = time.time_ns()
                for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgba"), skip_nans=True):
                    x, y, z = p[0], p[1], p[2]
                    if len(p) > 3:
                        rgb = p[3]
                        r_value = (rgb & 0x00FF0000) >> 16
                        g_value = (rgb & 0x0000FF00) >> 8
                        b_value = rgb & 0x000000FF
                        arrayR.update(r_value)
                        arrayG.update(g_value)
                        arrayB.update(b_value)

                    arrayT.update(int(str(t)))
                    arrayX.update(x)
                    arrayY.update(y)
                    arrayZ.update(z)

                totalArrayTime += time.time_ns() - arrayTimeStart
                writeToFile(arrayX, arrayY, arrayZ, arrayT, arrayR, arrayG, arrayB)
                arrayX, arrayY, arrayZ, arrayT, arrayR, arrayG, arrayB = createArrs()
                f.write(str(t) + "\n")

    print("Total points: " + str(totalNumPoints))
    endTime = time.time_ns()
    print("Total time used = " + str((endTime - startTime) * 1e-9))
    print("Array time used = " + str(totalArrayTime * 1e-9))
    result = {
        "numFiles": outFileCount,
        "numPoints": totalNumPoints,
        "totalTimeUsed": str((endTime - startTime) * 1e-9),
        "arrayTimeUsed": str(totalArrayTime * 1e-9),
        "totalMessages": count + 1,
        "size": utils.getFolderSize(pathOut),
    }
    return result
