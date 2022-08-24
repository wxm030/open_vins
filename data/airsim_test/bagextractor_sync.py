#!/usr/bin/env python
print "importing libraries"

import cv2
import csv
import os
import sys
import argparse
import rosbag
import rospy
from cv_bridge import CvBridge
import shutil

try:
    import cv
    png_flag = cv.CV_IMWRITE_PNG_COMPRESSION
except ImportError:    
    png_flag = cv2.IMWRITE_PNG_COMPRESSION


#setup the argument list
parser = argparse.ArgumentParser(description='Extract a ROS bag containing a image and imu topics.')
parser.add_argument('--bag', metavar='bag', help='ROS bag file')
parser.add_argument('--image-topics',  metavar='image_topics', nargs='+', help='Image topics %(default)s')
parser.add_argument('--imu-topics',  metavar='imu_topics', nargs='+', help='Imu topics %(default)s')
parser.add_argument('--gt_topic',  metavar='gt_topic', nargs='+', help='GT topics %(default)s')
parser.add_argument('--output-folder',  metavar='output_folder', nargs='?', default="output", help='Output folder %(default)s')
parser.add_argument('--sync_stereo',  metavar='sync_stereo', nargs='?', default="False", help='sync stereo flag')

#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

if parsed.image_topics is None and parsed.imu_topics is None:
    print "ERROR: Need at least one camera or IMU topic."
    sys.exit(-1)

#create output folder
try:
  os.makedirs(parsed.output_folder)
except:
  pass

#extract gt data
data_gt_csv_path = ""
if parsed.gt_topic is not None:
    os.makedirs("{0}/gt".format(parsed.output_folder))  
    with open( "{0}/gt/{1}".format(parsed.output_folder, "data.csv"), 'wb') as gtfile:
        spamwriter = csv.writer(gtfile, delimiter=',')
        spamwriter.writerow(["#timestamp", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]", "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []"])
        data_gt_csv_path = "{0}/gt/{1}".format(parsed.output_folder, "data.csv")


for topic, msg, t in rosbag.Bag(parsed.bag).read_messages():
    if topic in parsed.gt_topic:
        trans = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        rot = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        timestamp_int = t
        with open( data_gt_csv_path, 'a') as gtfile:
            spamwriter = csv.writer(gtfile, delimiter=',')
            spamwriter.writerow([timestamp_int, trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]])

#extract imu data
data_imu_csv_path = dict()
if parsed.imu_topics is not None:
    for iidx, topic in enumerate(parsed.imu_topics):
        os.makedirs("{0}/imu{1}".format(parsed.output_folder, iidx))  
        with open( "{0}/imu{1}/{2}".format(parsed.output_folder, iidx, "data.csv"), 'wb') as imufile:
            spamwriter = csv.writer(imufile, delimiter=',')
            spamwriter.writerow(["#timestamp [ns]","w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z[rad s^-1]", "a_RS_S_x[m s^-2]", "a_RS_S_y [m	s^-2]", "a_RS_S_z [m s^-2]"])
            data_imu_csv_path[topic] = "{0}/imu{1}/{2}".format(parsed.output_folder, iidx, "data.csv")

for topic, msg, t in rosbag.Bag(parsed.bag).read_messages():
    if topic in parsed.imu_topics:
        omega = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        alpha = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        timestamp_int = t
        with open( data_imu_csv_path[topic], 'a') as imufile:
            spamwriter = csv.writer(imufile, delimiter=',')
            spamwriter.writerow([timestamp_int, omega[0],omega[1],omega[2], alpha[0],alpha[1],alpha[2] ])


#extract images to folder
cam_data_path = dict()
t_cam_all = dict()
cam_img_all = dict()
if parsed.image_topics is not None:
    for cidx, topic in enumerate(parsed.image_topics):
        os.makedirs("{0}/cam{1}".format(parsed.output_folder, cidx))  
        os.makedirs("{0}/cam{1}/data".format(parsed.output_folder, cidx))  
        with open( "{0}/cam{1}/{2}".format(parsed.output_folder, cidx, "data.csv"), 'wb') as cam_csv:
            spamwriter_cam = csv.writer(cam_csv, delimiter=',')
            spamwriter_cam.writerow(["timestamp", "filename"])
            cam_data_path[topic] = "{0}/cam{1}".format(parsed.output_folder, cidx)
            t_cam_all[topic] = list()
            cam_img_all[topic] = list()

bridge = CvBridge()
for topic, msg, t in rosbag.Bag(parsed.bag).read_messages():
    if topic in parsed.image_topics:
        timestamp = msg.header.stamp
        params = list()
        params.append(png_flag)
        params.append(0) #0: loss-less  
        filename_img = "{0}{1:09d}.png".format(timestamp.secs, timestamp.nsecs)

        t_cam_all[topic].append(t)
        cam_img_all[topic].append(filename_img)

        with open( cam_data_path[topic]+"/data.csv", 'a') as cam_csv:
            spamwriter_cam = csv.writer(cam_csv, delimiter=',')
            spamwriter_cam.writerow([timestamp, filename_img ])

        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite( "{0}/data/{1}".format(cam_data_path[topic], filename_img), cv_img, params )

if parsed.image_topics is not None and parsed.sync_stereo is True:
    cam_idx_t = dict()
    img_idx_path = dict()
    for cidx, topic in enumerate(parsed.image_topics):
        cam_idx_t[cidx] = t_cam_all[topic]
        img_idx_path[cidx] = cam_img_all[topic]
        os.makedirs("{0}/cam{1}/data_sync".format(parsed.output_folder, cidx))  

    for i in  range(len(cam_idx_t[1])):
        for t0 in cam_idx_t[0]:
            if abs(t0-cam_idx_t[1][i]) < rospy.Duration(secs=0.02):
                src = parsed.output_folder + "/cam1/data/" + img_idx_path[1][i]
                filename_img = "{0}{1:09d}.png".format(t0.secs, t0.nsecs)
                dst = parsed.output_folder + "/cam1/data_sync/" + filename_img
                shutil.copyfile(src, dst)
                
                
        
