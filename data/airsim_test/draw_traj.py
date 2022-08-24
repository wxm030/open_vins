#! /usr/bin/python
import os
import sys

import copy
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
_EPS = np.finfo(float).eps * 4.0  #8.881784197001252e-16
from scipy.spatial.transform import Rotation as R


def draw_main(str1, str2):
    #read all gps data
    f1 = open(str1, 'r')
    lines1 = f1.readline()  # skip the first line
    lines1 = f1.readlines()

    time_ns = []
    X   = []
    Y   = []
    Z   = []
    qx = []
    qy = []
    qz = []
    qw = []
    vx_b_vo = []
    vy_b_vo = []
    vz_b_vo = []

    for line in lines1:
        ll = line.split(',')
        if(ll[0] == '#'):
            continue
        time_ns.append(int(ll[0])/1e9)

        X.append(float(ll[1]))
        Y.append(float(ll[2]))
        Z.append(float(ll[3]))

        qw.append(float(ll[4]))
        qx.append(float(ll[5]))
        qy.append(float(ll[6]))
        qz.append(float(ll[7]))


    #read all vio data
    f2 = open(str2, 'r')
    lines2 = f2.readline()  # skip the first line
    lines2 = f2.readlines()

    time_ns_estimate = []
    X_estimate  = []
    Y_estimate  = []
    Z_estimate  = []
    qx_estimate = []
    qy_estimate = []
    qz_estimate = []
    qw_estimate = []
    vx_b_vo_estimate = []
    vy_b_vo_estimate = []
    vz_b_vo_estimate = []

    for line in lines2:
        ll = line.split(',')
        if(ll[0] == '#'):
            continue
        time_ns_estimate.append(int(ll[0])/1e9)
        X_estimate.append(float(ll[1]))
        Y_estimate.append(float(ll[2]))
        Z_estimate.append(float(ll[3]))

        qw_estimate.append(float(ll[4]))
        qx_estimate.append(float(ll[5]))
        qy_estimate.append(float(ll[6]))
        qz_estimate.append(float(ll[7]))

    print(len(time_ns))
    print(len(time_ns_estimate))


    #draw 3D traj
    fig = plt.figure(1);
    ax = fig.add_subplot(111, projection='3d');
    ax.plot(X, Y, Z,color='green', label='gps')
    ax.plot(X_estimate, Y_estimate, Z_estimate, color='red', label='vio')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    #draw x,y,z 
    fig2, (ax1, ax2, ax3) = plt.subplots(3, sharey=False)

    # trans x
    ax1.plot(time_ns, X, 'g--', label='x_gps')
    ax1.plot(time_ns_estimate, X_estimate, 'r--', label='x_vio')
    ax1.set_xlabel('',fontsize=20)
    ax1.set_ylabel('m',fontsize=20)
    ax1.legend(loc=1, ncol=3, shadow=True)
    ax1.grid(True)

    # trans y
    ax2.plot(time_ns, Y, 'g--', label='y_gps')
    ax2.plot(time_ns_estimate, Y_estimate, 'r--', label='y_vio')
    ax2.set_xlabel('',fontsize=20)
    ax2.set_ylabel('m',fontsize=20)
    ax2.legend(loc=1, ncol=3, shadow=True)
    ax2.grid(True)

    # trans z
    ax3.plot(time_ns, Z, 'g--', label='z_gps')
    ax3.plot(time_ns_estimate, Z_estimate, 'r--', label='z_vio')
    ax3.set_xlabel('',fontsize=20)
    ax3.set_ylabel('m',fontsize=20)
    ax3.legend(loc=1, ncol=3, shadow=True)
    ax3.grid(True)

    plt.legend(loc=0)
    # plt.savefig("1.png")

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("draw_trajectory.py traj_gps.txt  traj_vio.txt")
        exit(0)
        
    draw_main(sys.argv[1], sys.argv[2])
