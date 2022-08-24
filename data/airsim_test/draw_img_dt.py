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
    dt_ms   = []

    for line in lines1:
        ll = line.split(',')
        if(ll[0] == '#'):
            continue;
        time_ns.append(float(ll[0]))


    for i in range(len(time_ns)):
        if i > 0 :
            dt = (time_ns[i] - time_ns[i-1])/1000000 # ms
            if dt > 200:
                print(dt)
                print(time_ns[i])
            dt_ms.append(dt)

    x_dt = []
    for i in range(len(dt_ms)):
        x_dt.append(i)

    print(len(dt_ms))
    print(len(time_ns))


    # #about rotation
    fig, (ax1) = plt.subplots(1, sharey=False)

    # #img dt
    ax1.plot(x_dt, dt_ms, 'g-', label='img dt')
    ax1.set_xlabel('',fontsize=20)
    ax1.set_ylabel('ms',fontsize=20)
    ax1.set_ylim(0, 1000)
    ax1.legend(loc=1, ncol=3, shadow=True)
    ax1.grid(True)

    fig.suptitle('img dt info', fontsize=24)   
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("draw_imu.py imu_data.csv")
        exit(0)

    draw_main(sys.argv[1])
