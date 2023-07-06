import json
import os
import argparse
import rosbag
import rospy
from reverseIMU import reverseIMU
from reversePointcloud import reversePointcloud
from reverseImage import reverseImage
import multiprocessing 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Script for reversing result files from rosbagutils API.\
            \nOrganize the result files following the instructions below.\
            \nGive the path to this folder in the --datapath argument."
    )

    parser.add_argument("config_file", help="json config file")
    parser.add_argument(
        "--in_docker",
        default=False,
        help="flag to indicate whether you are inside docker, this will add a /data prefix to the your file path",
    )
    args = parser.parse_args()
    f = open(args.config_file)
    configs = json.load(f)
    print(configs)
    
    if args.in_docker:
        folder_path = "/data/" + configs["datapath"]
    else:
        folder_path = configs["datapath"]
        folder_path = os.path.abspath(folder_path)
    print("Processing data at: ", folder_path)
    
    lidar_path = os.path.join(folder_path, "lidar/")
    imu_path = os.path.join(folder_path, "imu/")
    image_path = os.path.join(folder_path, "cam_0/")

    process1 = multiprocessing.Process(target=reversePointcloud, 
                                       args=(lidar_path, configs["folderType"]["lidar"]["topicName"], configs["folderType"]["lidar"]["frame_id"]))
    process2 = multiprocessing.Process(target=reverseIMU, 
                                       args=(imu_path, configs["folderType"]["imu"]["topicName"], configs["folderType"]["imu"]["frame_id"]))
    process3 = multiprocessing.Process(target=reverseImage, 
                                       args=(image_path, configs["folderType"]["cam_0"]["topicName"], configs["folderType"]["cam_0"]["frame_id"]))
    process1.start()
    process2.start()
    process3.start()
