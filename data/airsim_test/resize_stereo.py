#!/usr/bin/python3
from PIL import Image
import PIL.Image as PILImage
import numpy as np
import sys
import os
import cv2


def get_tick_sort(str):
    tmp = str.split(',')[0]
    return int(tmp.split('.')[0])


def resizeImage(filename):
    img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    if ( img.shape[0] == 512):
    	return
    imgCropped=img[32:736,160:864]
    # print('original dimensions : ', img.shape)
    scale = 1.375  # percent of original size
    ori_width = imgCropped.shape[1]
    ori_height = imgCropped.shape[0]

    width = int(imgCropped.shape[1]/scale)
    height = int(imgCropped.shape[0] / scale)
    dim = (width, height)
    # resize image
    resized = cv2.resize(imgCropped, dim)#, interpolation=cv2.INTER_AREA
    # print('resized dimensions : ', resized.shape)
    save_file = filename  # "1.png"  #
    cv2.imwrite(save_file, resized)
    # cv2.imshow("resized image", resized)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


def resizeImageDir(file_dir):
    print("-> Scanning " + file_dir)
    img_files = sorted(os.listdir(file_dir), key=get_tick_sort)
    for img_file in img_files:
        full_image_file = file_dir + img_file
        print("img_file: ", full_image_file)
        resizeImage(full_image_file)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("draw_imu.py left_base_dir right_base_dir")
        exit(0)

    left_dir = sys.argv[1] + "/data/"
    right_dir = sys.argv[2] + "/data/"
    print("left_dir: ", left_dir)
    print("right_dir: ", right_dir)
    resizeImageDir(left_dir)
    resizeImageDir(right_dir)
