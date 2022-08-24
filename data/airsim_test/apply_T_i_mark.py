#!/usr/bin/python

"""
This script apply T_i_mark to the ground truth trajectory.
"""

import sys
import os
import numpy
import argparse
import pyquaternion
from scipy.spatial.transform import Rotation as R

def read_file_list(filename):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''This script apply T_i_mark to the groundtruth file. ''')
    parser.add_argument('input_file', help='original trajectory (format: #timestamp [s] p_RS_R_x [m] p_RS_R_y [m] p_RS_R_z [m] q_RS_w [] q_RS_x [] q_RS_y [] q_RS_z [])')
    parser.add_argument('--timeshift', help='time offset added to the timestamps of the input file (default: 0.0)',default=0.0)
    args = parser.parse_args()

    print("reading the input data ...")
    first_list = read_file_list(args.input_file)

    first_pose = numpy.matrix([[float(value) for value in first_list[a][:]] for a in first_list.keys()]).transpose()

    T_x = numpy.matrix(numpy.zeros((4, 4)))
    R_x = pyquaternion.Quaternion(array=[0.5006342249978324, -0.49748077650605057, -0.501908559163185, 0.49996604685367637]).transformation_matrix # w,x,y,z
    T_x[0:3, 0:3] = R_x[0:3, 0:3]
    T_x[0:3, 3] = numpy.array([[0.010614696293886584],[0.002210473301017582],[-0.015427063642575331]])
    T_x[3, 3] = 1.0

    print(T_x)
    # track alignment(translation)
    first_xyz_aligned = numpy.matrix(numpy.zeros((3, numpy.size(first_pose[0]))))
    first_quat_aligned = numpy.matrix(numpy.zeros((4, numpy.size(first_pose[0]))))

    T_0 = numpy.matrix(numpy.zeros((4, 4)))
    R_0 = pyquaternion.Quaternion(array=[first_pose[3, 0], first_pose[4, 0], first_pose[5, 0],first_pose[6, 0]]).transformation_matrix # w,x,y,z
    T_0[0:3, 0:3] = R_0[0:3, 0:3]
    T_0[0:3, 3] = first_pose[0:3, 0]
    T_0[3, 3] = 1.0

    for i in range(numpy.size(first_pose[0])):

        T_i = numpy.matrix(numpy.zeros((4, 4)))
        R_i = pyquaternion.Quaternion(array=[first_pose[3, i], first_pose[4, i], first_pose[5, i],first_pose[6, i]]).transformation_matrix # w,x,y,z
        T_i[0:3, 0:3] = R_i[0:3, 0:3]
        T_i[0:3, 3] = first_pose[0:3, i]
        T_i[3, 3] = 1.0

        #T_new = numpy.dot(T_x, T_i)
        T_new = numpy.dot(numpy.dot(numpy.dot(T_x, T_0.I),T_i),T_x.I)

        rot = T_new[0:3, 0:3]
        trans = T_new[0:3, 3]


        first_xyz_aligned[0:3, i] = trans
        first_quat_aligned[0:4, i] = numpy.matrix(list(pyquaternion.Quaternion(matrix=rot))).transpose()


    # save allignment result
    file_name = os.path.join("TAS_" + os.path.basename(args.input_file))
    file = open(file_name, "w")
    file.write("#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z [])\n")
    file.write("\n".join(["%d,%f,%f,%f,%f,%f,%f,%f"%((a+float(args.timeshift))*1e9,x,y,z,qx,qy,qz,qw) for (a),(x,y,z),(qx,qy,qz,qw) in zip(first_list.keys(),first_xyz_aligned.transpose().A,first_quat_aligned.transpose().A)]))
    file.close()

    


        
