#!/usr/bin/python3
import os
import sys
import shutil
import threading
import os.path as path
from PIL import Image
import PIL.Image as PILImage
import numpy as np


def get_tick(str):
    return int(str.split(',')[0])


def chk_mkdir(dir_path):
    if not (path.exists(dir_path)):
        os.mkdir(dir_path)


def get_tick_sort(str):
    tmp = str.split(',')[0]
    return int(tmp.split('.')[0])


def yuv_cvt2png(yuv_filename, png_path):
    # image format is ".yuv"
    if os.path.splitext(yuv_filename)[1] == ".yuv" or os.path.splitext(yuv_filename)[1] == ".nv21":
        f_y = open(yuv_filename, 'rb')
        width = 1024
        height = 768
        # image_out = PILImage.new("L", (width, height))
        # pimg = image_out.load()
        # for i in range(0,height):
        #     for j in range(0, width):
        #         pimg[j, i] = int(ord(f_y.read(1)))
        # 实际只有Y分量
        img_data = f_y.read(width * height)
        image_out = Image.frombuffer('L', [width, height],
                                     img_data, 'raw', 'L', 0, 1)
        image_out.save(png_path, 'PNG')
        image_out.close()


def p5pgm_cvt2png(s16pgm_path, png_path):
    s16pgm = open(s16pgm_path, 'rb')
    assert s16pgm.readline() == b'P5\n'
    (img_cols, img_rows) = [int(i) for i in s16pgm.readline().split()]
    depth = int(s16pgm.readline())
    scale = 1
    if depth > 255:
        scale = 2
    img_data = s16pgm.read(img_cols * scale * img_rows)
    img = Image.frombuffer('L', [img_cols * scale, img_rows],
                           img_data, 'raw', 'L', 0, 1)
    img.save(png_path, 'PNG')
    img.close()
    s16pgm.close()


gcamx_gray = False


def p5c3pgm_cvt2png(p5c3pgm_path, png_path):
    p5c3pgm = open(p5c3pgm_path, 'rb')
    # assert p5c3pgm.readline() == b'P5\n'
    assert p5c3pgm.readline() == b'P6\n'
    (img_cols, img_rows) = [int(i) for i in p5c3pgm.readline().split()]
    depth = int(p5c3pgm.readline())
    img_data = bytearray(p5c3pgm.read(img_cols * img_rows * 3))
    for i in range(0, len(img_data), 3):
        img_data[i], img_data[i+1], img_data[i +
                                             2] = img_data[i+2], img_data[i+1], img_data[i]
    img = Image.frombuffer('RGB', [img_cols, img_rows],
                           bytes(img_data), 'raw', 'RGB', 0, 1)
    if gcamx_gray:
        gray = img.convert('L')
        gray.save(png_path, 'PNG')
        gray.close()
    else:
        img.save(png_path, 'PNG')
    img.close()
    p5c3pgm.close()


class Controller:
    def __init__(self, argv):
        self.cpy_flag = False
        self.src_path = path.abspath(argv[1])
        self.beg_tick = int(argv[2])
        self.end_tick = int(argv[3])
        if 5 == len(argv):
            self.dst_path = path.abspath(argv[4])
            chk_mkdir(self.dst_path)
            if os.path.exists(self.dst_path):
                self.cpy_flag = True
            else:
                print(self.dst_path + "in un-available, please check again.")
                exit(1)
        else:
            self.dst_path = ''


class ImgDataProc (threading.Thread):
    def __init__(self, data_path, dst_data_path, ctrl, src_s16=0, type_cvt=1, nchn=1):
        threading.Thread.__init__(self)
        self.data_path = data_path
        self.beg_tick = ctrl.beg_tick
        self.end_tick = ctrl.end_tick
        self.cpy_flag = ctrl.cpy_flag
        self.dst_data_path = dst_data_path
        self.cvt_type = type_cvt
        self.src_s16 = src_s16
        self.chan_num = nchn

    def run(self):
        print("-> Scanning " + self.data_path)
        img_files = sorted(os.listdir(self.data_path), key=get_tick_sort)
        for img_file in img_files:
            img_tick = int(img_file.split('.')[0])
            if img_tick >= self.beg_tick and img_tick <= self.end_tick:
                img_src_path = path.join(self.data_path, img_file)
                if path.getsize(img_src_path):
                    if img_file.split('.')[1] == 'png' or self.cvt_type == 0:
                        if self.cpy_flag:
                            shutil.copy(img_src_path, self.dst_data_path)
                        else:
                            shutil.move(img_src_path, self.dst_data_path)
                    else:
                        img_dst_path = path.join(self.dst_data_path,
                                                 img_file.split('.')[0] +
                                                 '.png')
                        if img_file.split('.')[1] == 'yuv' or img_file.split('.')[1] == 'nv21':
                            yuv_cvt2png(img_src_path, img_dst_path)
                            print(img_src_path)
                        elif not(self.src_s16) and (3 != self.chan_num):
                            img = Image.open(img_src_path)
                            img.save(img_dst_path, 'PNG')
                            img.close()
                        elif (1 == self.chan_num):
                            p5pgm_cvt2png(img_src_path, img_dst_path)
                        else:
                            p5c3pgm_cvt2png(img_src_path, img_dst_path)

            if img_tick >= self.end_tick:
                break
        if not(self.cpy_flag):
            if (os.path.exists(self.data_path)):
                print("+> Removing " + self.data_path)
                shutil.rmtree(self.data_path, False)


class MsgDataProc:
    def __init__(self, ctrl, beg_shift=500 * 1000 * 1000,
                 end_shift=500 * 1000 * 1000):
        self.beg_tick = ctrl.beg_tick - beg_shift
        self.end_tick = ctrl.end_tick + end_shift

    def process(self, data_csv, dst_data_csv):
        csv_data = ""
        csv_read = open(data_csv, 'r')
        line = csv_read.readline()
        csv_data += line
        for line in csv_read:
            line_sep = line.split(',')
            msg_tick = int(line_sep[0])
            if msg_tick >= self.beg_tick and msg_tick <= self.end_tick:
                csv_data += line
            if msg_tick > self.end_tick:
                break
        csv_read.close()
        csv_write = open(dst_data_csv, 'w+')
        csv_write.write(csv_data)
        csv_write.flush()
        csv_write.close()

    def generate(self, est_csv, att_csv, ned_csv):
        csv_data = \
            "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], " \
            "q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], " \
            "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], " \
            "b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], " \
            "b_w_RS_S_z [rad s^-1], " \
            "b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n"
        att_read = open(att_csv, 'r')
        ned_read = open(ned_csv, 'r')
        att_read.readline()  # skip first line
        ned_read.readline()  # skip first line
        att_data = att_read.readline()
        ned_data = ned_read.readline()
        att_tick = get_tick(att_data)
        ned_tick = get_tick(ned_data)
        reach_end = False

        if att_tick > ned_tick:
            while att_tick > ned_tick:
                ned_data = ned_read.readline()
                if len(ned_data):
                    ned_tick = get_tick(ned_data)
                else:
                    reach_end = True
                    break

        if not reach_end:
            for att_data in att_read:
                att_tick = get_tick(att_data)
                if att_tick > ned_tick:
                    while att_tick > ned_tick:
                        ned_data = ned_read.readline()
                        if len(ned_data):
                            ned_tick = get_tick(ned_data)
                        else:
                            reach_end = True
                            break  # @by hourongbo
                if reach_end:
                    break
                att_elems = att_data.split(',')
                ned_elems = ned_data.split(',')
                est_data = \
                    att_elems[0] + ',' + ned_elems[1] + ',' + \
                    ned_elems[2] + ',' + ned_elems[3] + ',' + \
                    att_elems[1] + ',' + att_elems[2] + ',' + \
                    att_elems[3] + ',' + \
                    att_elems[4].replace("\n", "") + ',' + \
                    ned_elems[4] + ',' + ned_elems[5] + ',' + \
                    ned_elems[6] + ',' + "0, 0, 0, 0, 0, 0\n"
                csv_data += est_data
            att_read.close()
            ned_read.close()
            csv_write = open(est_csv, 'w+')
            csv_write.write(csv_data)
            csv_write.flush()
            csv_write.close()


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("euroc.py dataset_path begin_tick end_tick [new_path]")
        exit(0)
    ctrl = Controller(sys.argv)
    msg_tweaker = MsgDataProc(ctrl)

    mav_path = path.join(ctrl.src_path, 'mav0')
    if not(path.exists(mav_path)):
        print("ERROR: Path " + mav_path + " doesn't exists")
        exit(0)
    if ctrl.cpy_flag:
        dst_mav_path = path.join(ctrl.dst_path, 'mav0')
        chk_mkdir(dst_mav_path)

    has_ned = False
    has_imu = False

    for sub_dir in os.listdir(mav_path):
        if 'cam' == sub_dir[0:3]:
            cam_root = path.join(mav_path, sub_dir)
            print("Processing " + cam_root)
            if ctrl.cpy_flag:
                dst_cam_root = path.join(dst_mav_path, sub_dir)
                chk_mkdir(dst_cam_root)

            # process images in data-* folders
            if ctrl.cpy_flag:
                dst_data_path = path.join(dst_cam_root, 'data')
            else:
                dst_data_path = path.join(cam_root, 'data')
            chk_mkdir(dst_data_path)

            nchannel = 1
            if sub_dir == 'camx':
                nchannel = 3
            # else:
            #     continue
            data_proc = []
            for data_dir in os.listdir(cam_root):
                if data_dir[0:5] == 'data-':
                    data_dir = path.join(cam_root, data_dir)
                    cur_thread = ImgDataProc(
                        data_dir, dst_data_path, ctrl, 0, 1, nchannel)
                    cur_thread.start()
                    data_proc.append(cur_thread)
                    # cur_thread.join()
            for thread in data_proc:
                thread.join()

            # process timestamp exposure data sheet expo.csv
            expo_csv = path.join(cam_root, 'expo.csv')
            if os.path.exists(expo_csv):
                print("-> Verifying " + expo_csv)
                csv_expo = ''
                expo_read = open(expo_csv, 'r')
                line_head = expo_read.readline()
                csv_expo += line_head
                for line in expo_read:
                    line_sep = (line.split())[0].split(',')
                    img_tick = int(line_sep[0])
                    img_path = path.join(dst_data_path, line_sep[0] + '.png')
                    if img_tick >= ctrl.beg_tick and img_tick <= ctrl.end_tick:
                        if path.exists(img_path):
                            csv_expo += line
                    if img_tick >= ctrl.end_tick:
                        break
                expo_read.close()
                if ctrl.cpy_flag:
                    dst_expo_csv = path.join(dst_cam_root, 'expo.csv')
                else:
                    dst_expo_csv = expo_csv
                expo_write = open(dst_expo_csv, 'w+')
                expo_write.write(csv_expo)
                expo_write.flush()
                expo_write.close()

            # process file-timestamp and file-name data.csv
            data_csv = path.join(cam_root, 'data.csv')
            print("-> Verifying " + data_csv)
            csv_data = ''
            data_read = open(data_csv, 'r')
            line_head = data_read.readline()
            csv_data += line_head
            for line in data_read:
                line_sep = (line.split())[0].split(',')
                img_tick = int(line_sep[0])
                img_path = path.join(dst_data_path, line_sep[0] + '.png')
                if img_tick >= ctrl.beg_tick and img_tick <= ctrl.end_tick:
                    if path.exists(img_path):
                        csv_data += line_sep[0] + ',' + line_sep[0] + '.png\n'
                if img_tick >= ctrl.end_tick:
                    break
            data_read.close()
            if ctrl.cpy_flag:
                dst_data_csv = path.join(dst_cam_root, 'data.csv')
            else:
                dst_data_csv = data_csv
            data_write = open(dst_data_csv, 'w+')
            data_write.write(csv_data)
            data_write.flush()
            data_write.close()

        elif 'dpt' == sub_dir[0:3]:
            dpt_root = path.join(mav_path, sub_dir)
            print("Processing " + dpt_root)
            if ctrl.cpy_flag:
                dst_dpt_root = path.join(dst_mav_path, sub_dir)
                chk_mkdir(dst_dpt_root)

            # process images in data-* folders
            if ctrl.cpy_flag:
                dst_data_path = path.join(dst_dpt_root, 'data')
            else:
                dst_data_path = path.join(dpt_root, 'data')
            chk_mkdir(dst_data_path)

            data_proc = []
            for data_dir in os.listdir(dpt_root):
                if data_dir[0:5] == 'data-':
                    data_dir = path.join(dpt_root, data_dir)
                    cur_thread = ImgDataProc(data_dir, dst_data_path, ctrl, 1)
                    cur_thread.start()
                    data_proc.append(cur_thread)
                    # cur_thread.join()
            for thread in data_proc:
                thread.join()

            # process file-timestamp and file-name data.csv
            data_csv = path.join(dpt_root, 'data.csv')
            print("-> Verifying " + data_csv)
            csv_data = ''
            data_read = open(data_csv, 'r')
            line_head = data_read.readline()
            csv_data += line_head
            for line in data_read:
                line_sep = (line.split())[0].split(',')
                img_tick = int(line_sep[0])
                if ctrl.beg_tick <= img_tick and img_tick <= ctrl.end_tick:
                    img_path = path.join(dst_data_path, line_sep[0] + '.pgm')
                    if path.exists(img_path):
                        csv_data += line_sep[0] + ',' + line_sep[0] + '.pgm\n'
                if img_tick > ctrl.end_tick:
                    break
            data_read.close()
            if ctrl.cpy_flag:
                dst_data_csv = path.join(dst_dpt_root, 'data.csv')
            else:
                dst_data_csv = data_csv
            data_write = open(dst_data_csv, 'w+')
            data_write.write(csv_data)
            data_write.flush()
            data_write.close()

        elif 'imu' == sub_dir[0:3]:
            imu_root = path.join(mav_path, sub_dir)
            print("Processing " + imu_root)
            imu_csv = path.join(imu_root, 'data.csv')
            att_csv = path.join(imu_root, 'fcs_att.csv')
            if ctrl.cpy_flag:
                dst_imu_root = path.join(dst_mav_path, sub_dir)
                dst_imu_csv = path.join(dst_imu_root, 'data.csv')
                dst_att_csv = path.join(dst_imu_root, 'fcs_att.csv')
                chk_mkdir(dst_imu_root)
            else:
                dst_imu_csv = imu_csv
                dst_att_csv = att_csv
            msg_tweaker.process(imu_csv, dst_imu_csv)
            if os.path.exists(att_csv):
                msg_tweaker.process(att_csv, dst_att_csv)
            has_imu = True

        elif 'gps' == sub_dir[0:3]:
            gps_root = path.join(mav_path, sub_dir)
            print("Processing " + gps_root)
            gps_csv = path.join(gps_root, 'data.csv')
            if ctrl.cpy_flag:
                dst_gps_root = path.join(dst_mav_path, sub_dir)
                dst_gps_csv = path.join(dst_gps_root, 'data.csv')
                chk_mkdir(dst_gps_root)
            else:
                dst_gps_csv = gps_csv
            msg_tweaker.process(gps_csv, dst_gps_csv)

        elif 'rtk' == sub_dir[0:3]:
            rtk_root = path.join(mav_path, sub_dir)
            print("Processing " + rtk_root)
            rtk_csv = path.join(rtk_root, 'data.csv')
            if ctrl.cpy_flag:
                dst_rtk_root = path.join(dst_mav_path, sub_dir)
                dst_rtk_csv = path.join(dst_rtk_root, 'data.csv')
                chk_mkdir(dst_rtk_root)
            else:
                dst_rtk_csv = rtk_csv
            msg_tweaker.process(rtk_csv, dst_rtk_csv)

        elif 'ned' == sub_dir[0:3]:
            ned_root = path.join(mav_path, sub_dir)
            print("Processing " + ned_root)
            ned_csv = path.join(ned_root, 'data.csv')
            if ctrl.cpy_flag:
                dst_ned_root = path.join(dst_mav_path, sub_dir)
                dst_ned_csv = path.join(dst_ned_root, 'data.csv')
                chk_mkdir(dst_ned_root)
            else:
                dst_ned_csv = ned_csv
            msg_tweaker.process(ned_csv, dst_ned_csv)
            has_ned = True

        elif 'attx' == sub_dir:
            attx_root = path.join(mav_path, sub_dir)
            print("Processing " + attx_root)
            attx_csv = path.join(attx_root, 'data.csv')
            if ctrl.cpy_flag:
                dst_attx_root = path.join(dst_mav_path, sub_dir)
                dst_attx_csv = path.join(dst_attx_root, 'data.csv')
                chk_mkdir(dst_attx_root)
            else:
                dst_attx_csv = attx_csv
            msg_tweaker.process(attx_csv, dst_attx_csv)

        # copy sensor.yaml
        if ctrl.cpy_flag:
            yaml_path = path.join(mav_path, sub_dir, 'sensor.yaml')
            dst_yaml_path = path.join(dst_mav_path, sub_dir, 'sensor.yaml')
            if os.path.exists(yaml_path):
                shutil.copy(yaml_path, dst_yaml_path)

    if has_ned and has_imu:
        # generate state_groundtruth_estimate0
        if ctrl.cpy_flag:
            est_root = path.join(dst_mav_path, 'state_groundtruth_estimate0')
            ned_csv = path.join(dst_mav_path, 'ned', 'data.csv')
            att_csv = path.join(dst_mav_path, 'imu0', 'fcs_att.csv')
        else:
            est_root = path.join(mav_path, 'state_groundtruth_estimate0')
            ned_csv = path.join(mav_path, 'ned', 'data.csv')
            att_csv = path.join(mav_path, 'imu0', 'fcs_att.csv')
        print("Generating " + est_root)
        chk_mkdir(est_root)
        est_csv = path.join(est_root, 'data.csv')
        msg_tweaker.generate(est_csv, att_csv, ned_csv)
        # generate yaml file
        yml_data = "%YAML:1.0\n" + \
                   "sensor_type: FCS\n" + \
                   "comment: FCS estimated state\n" + \
                   "T_BS:\n" + \
                   "    cols: 4\n" + \
                   "    rows: 4\n" + \
                   "    data: [1.0, 0.0, 0.0, 0.0,\n" + \
                   "                 0.0, 1.0, 0.0, 0.0,\n" + \
                   "                 0.0, 0.0, 1.0, 0.0,\n" + \
                   "                 0.0, 0.0, 0.0, 1.0]\n"
        yaml_path = path.join(est_root, "sensor.yaml")
        yml_writer = open(yaml_path, 'w+')
        yml_writer.write(yml_data)
        yml_writer.flush()
        yml_writer.close()
