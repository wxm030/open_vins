#! /usr/bin/python
import os
import sys
import matplotlib.pyplot as plt
import math
import numpy as np


def draw_main(str):
    f1 = open(str, 'r')
    lines1 = f1.readline()  # skip the first line
    lines1 = f1.readlines()
    # std_t std_r std_v std_bg std_ba
    time_ns = []
    gyr_x   = []
    gyr_y   = []
    gyr_z   = []
    acc_x   = []
    acc_y   = []
    acc_z   = []
    dt_ms   = []

    for line in lines1:
        ll = line.split(',')
        if(ll[0] == '#'):
            continue;
        time_ns.append(float(ll[0]))
        gyr_x.append(float(ll[1]))
        gyr_y.append(float(ll[2]))
        gyr_z.append(float(ll[3]))
        acc_x.append(float(ll[4]))
        acc_y.append(float(ll[5]))
        acc_z.append(float(ll[6]))

    x = []
    for i in range(len(time_ns)):
        x.append(i)

    for i in range(len(time_ns)):
        if i > 0 :
            dt = (time_ns[i] - time_ns[i-1])/1000000 # ms
            dt_ms.append(dt)

    x_dt = []
    for i in range(len(dt_ms)):
        x_dt.append(i)

    print(len(dt_ms))
    print(len(time_ns))


    # #about rotation
    fig, (ax1, ax2, ax3) = plt.subplots(3, sharey=False)

    # #imu dt
    ax1.plot(x_dt, dt_ms, 'g-', label='imu dt')
    ax1.set_xlabel('',fontsize=20)
    ax1.set_ylabel('ms',fontsize=20)
    ax1.legend(loc=1, ncol=3, shadow=True)
    ax1.grid(True)

    # gyr
    ax2.plot(x, gyr_x, 'r-', label='gyr x')
    ax2.plot(x, gyr_y, 'g-', label='gyr y')
    ax2.plot(x, gyr_z, 'b-', label='gyr z')

    ax2.set_xlabel('',fontsize=20)
    ax2.set_ylabel('rad/s',fontsize=20)
    ax2.legend(loc=1, ncol=3, shadow=True)
    ax2.grid(True)

    # acc
    ax3.plot(x, acc_x, 'r-', label='acc x')
    ax3.plot(x, acc_y, 'g-', label='acc y')
    ax3.plot(x, acc_z, 'b-', label='acc z')

    ax3.set_xlabel('',fontsize=20)
    ax3.set_ylabel('m/s^2',fontsize=20)
    ax3.legend(loc=1, ncol=3, shadow=True)
    ax3.grid(True)
    fig.suptitle('imu info', fontsize=24)   
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("draw_imu.py imu_data.csv")
        exit(0)

    draw_main(sys.argv[1])
