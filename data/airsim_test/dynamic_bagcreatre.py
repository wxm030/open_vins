#!/usr/bin/env python
print("importing libraries")

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped  
from xml.dom.minidom import parse
import xml.dom.minidom
import sys, os
import argparse
import cv2
import PIL.Image as PILImage
import numpy as np
import csv

#setup the argument list
parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data. ')

parser.add_argument('--images_folder_cam0',  dest='images_folder_cam0', help='cam0 images from AR glass')
parser.add_argument('--images_folder_cam1',  dest='images_folder_cam1', help='cam1 images from AR glass')
parser.add_argument('--imu_csv',  dest='imu_csv', help='imu csv file from AR glass')
parser.add_argument('--output-bag', dest='output_bag',  default="output.bag", help='ROS bag file %(default)s')


#print help if no argument is specified
if len(sys.argv)<4:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()


def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', ".pgm", ".yuv"]:
                    image_files.append( os.path.join( path, f ) )
                    timestamps.append(int(os.path.splitext(f)[0]))
    #sort by timestamp
    sort_list = sorted(list(zip(timestamps, image_files)))
    image_files = [file[1] for file in sort_list]
    return image_files


def loadImageToRosMsg(filename):
    #image format is ".yuv"
    if os.path.splitext(filename)[1] == ".yuv":
        f_y = open(filename, 'rb')
        width = 640
        height = 400
        image_out = PILImage.new("L", (width, height))
        pimg = image_out.load()
        for i in range(0,height):
            for j in range(0, width):
                pimg[j, i] = int(ord(f_y.read(1)))

        image_np = np.array(image_out)
        image_np.reshape(height, width)
        timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
        timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )
        rosimage = Image()
        rosimage.header.stamp = timestamp
        rosimage.height = image_np.shape[0]
        rosimage.width = image_np.shape[1]
        rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
        rosimage.encoding = "mono8"
        rosimage.data = image_np.tostring()

    #image format is '.bmp', '.png', '.jpg', ".pgm"
    else:

        image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    
        timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
        timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )

        rosimage = Image()
        rosimage.header.stamp = timestamp
        rosimage.height = image_np.shape[0]
        rosimage.width = image_np.shape[1]
        rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
        rosimage.encoding = "mono8"
        rosimage.data = image_np.tostring()
    
    return rosimage, timestamp


def createImuMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    
    return rosimu, timestamp

def createAccGyroMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    acc_msg = Vector3Stamped()
    gryo_msg = Vector3Stamped()

    acc_msg.header.stamp = timestamp
    acc_msg.vector.x = float(omega[0])
    acc_msg.vector.y = float(omega[1])
    acc_msg.vector.z = float(omega[2])

    gryo_msg.header.stamp = timestamp
    gryo_msg.vector.x = float(alpha[0])
    gryo_msg.vector.y = float(alpha[1])
    gryo_msg.vector.z = float(alpha[2])

    return acc_msg, gryo_msg, timestamp


############################ main ###############################
#create the bag
acc_topic = "/acc0"
gyro_topic = "/gyr0"
imu_topic = "/imu0"
cam0_image_topic = "/cam0/image_raw"
cam1_image_topic = "/cam1/image_raw"

try:
    bag = rosbag.Bag(parsed.output_bag, 'w')
    
    #write images
    #cam0
    cam0dir = parsed.images_folder_cam0
    cam0_image_files = getImageFilesFromDir(cam0dir)
    for cam0_image_filename in cam0_image_files:
        image_msg, timestamp = loadImageToRosMsg(cam0_image_filename)
        bag.write(cam0_image_topic, image_msg, timestamp)
    #cam1
    cam1dir = parsed.images_folder_cam1
    cam1_image_files = getImageFilesFromDir(cam1dir)
    for cam1_image_filename in cam1_image_files:
        image_msg, timestamp = loadImageToRosMsg(cam1_image_filename)
        bag.write(cam1_image_topic, image_msg, timestamp)

    #read imu csv file and write imu msg
    imufile = parsed.imu_csv
    with open(imufile, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        headers = next(reader, None)
        for row in reader:
            #acc_msg, gyro_msg,timestamp = createAccGyroMessge(row[0], row[1:4], row[4:7])
            #bag.write(acc_topic, acc_msg, timestamp)
            #bag.write(gyro_topic, gyro_msg, timestamp)
            imu_msg,timestamp = createImuMessge(row[0], row[1:4], row[4:7])
            bag.write(imu_topic, imu_msg, timestamp)

finally:
    bag.close()

