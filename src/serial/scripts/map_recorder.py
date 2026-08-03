#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from datetime import datetime
from math import *
import shutil
import threading
import rospy
from geometry_msgs.msg import PointStamped
import string
import os, time
import sys, tty, select, termios

import numpy as np

import random
import matplotlib.pyplot as plt

import getch
import glob

import getpass

user = getpass.getuser()
if user == 'root':  # 도커에서 사용하기 위한 예외코드입니다..
    parent_dir = '/home/shh/'
else:
    parent_dir = os.path.join('/home', user)

TARGET_SAVE_PATH = os.path.join('/', 'test_f', 'src', 'serial', 'map')

GPS_TOPIC = '/Local/utm'


class MapRecorder:
    def __init__(self):
        rospy.init_node('map_recorder', anonymous=False)
        
        self.save_mode = 0
        self.callback_counter = 0
        self.settings = None
        self.utm = np.empty((0, 2), dtype=np.float32)
        self.total_saved = []
        self.cx, self.cy = [], []
        self.pre_x, self.pre_y = 0, 0

        self.topic_check_time = time.time()

        self.idx = 0
        self.sub = rospy.Subscriber(GPS_TOPIC, PointStamped, self.text_callback, queue_size=10)

        # count files in specific location
        file_list = [f for f in os.listdir(TARGET_SAVE_PATH) if os.path.isfile(os.path.join(TARGET_SAVE_PATH, f))]

        if len(file_list) != 0:

            # in order to determine the title of backup folder to first item of existing files.
            abs_path = os.path.join(TARGET_SAVE_PATH, file_list[0])
            new_folder_path = os.path.join(TARGET_SAVE_PATH, self.get_file_time(abs_path))
            rospy.logwarn(f'since there are files in save path, executing backup created to : {new_folder_path}')
            if not os.path.exists(new_folder_path):
                os.mkdir(new_folder_path)

            for f in file_list:
                shutil.move(os.path.join(TARGET_SAVE_PATH, f), os.path.join(new_folder_path, f))

        rospy.loginfo("#### #### MAP SAVER #### ####\n")
        rospy.loginfo("writing x,y for UTM")
        rospy.loginfo("Press 'P' to -pause-, 'S' to -Stop program-")

        self.exit_event = threading.Event()
        input_thread = threading.Thread(target=self.get_input, args=(self.exit_event, self.idx,))
        input_thread.daemon = True
        input_thread.start()

    def get_file_time(self, filepath):
        tm = os.path.getmtime(filepath)
        dt = datetime.fromtimestamp(tm)

        now = datetime.now()
        return dt.strftime("back_%y%m%d_%H%M%S")

    def get_input(self, exit_event, idx):
        # Set stdin to non-blocking mode
        self.settings = termios.tcgetattr(sys.stdin)

        tty.setcbreak(sys.stdin.fileno())

        while True:
            # Check if there is any input waiting on stdin
            if select.select([sys.stdin], [], [], 0)[0]:
                # Read a single character of input from stdin
                user_input = getch.getch()

                if user_input == 'p' or user_input == 'P':
                    if self.callback_counter != 0:
                        self.idx = self.idx + 1
                        self.total_saved.append(self.utm)
                        self.utm = np.empty((0, 2), dtype=np.float32)
                        rospy.loginfo(f'save {self.idx}.txt')
                    else:
                        print('\n')
                        rospy.logwarn('recording less than 1 sample is prohibited.')
                        continue

                elif user_input == 's' or user_input == 'S':
                    exit_event.set()
                    break

            if time.time() - self.topic_check_time > 3:
                if self.callback_counter == 0:
                    rospy.logwarn(f'i think The topic \'{GPS_TOPIC}\' I\'m subscribing is not that publishing anything. please check the topic again.')
                    self.topic_check_time = time.time()
        return

    def text_callback(self, msg):
        self.callback_counter = self.callback_counter + 1
        target_utm = np.array([msg.point.x, msg.point.y])

        print(f'\r recording : {self.idx + 1}.txt |  {target_utm}', end='')
        self.utm = np.append(self.utm, [target_utm], axis=0)

    def save_to_txt(self):
        for i, l in enumerate(self.total_saved):
            np.savetxt(os.path.join(TARGET_SAVE_PATH, f'{i+1}.txt'), l, delimiter=',', fmt='%.8f')
        rospy.loginfo('save done.')

    def plot(self, save_fig=True):
        for i, u in enumerate(self.total_saved):
            random.seed(time.time())
            r = random.random()
            g = random.random()
            b = random.random()
            random_rgb = (r, g, b)

            x_start = u[0, 0]
            x_end = u[-1, 0]
            y_start = u[0, 1]
            y_end = u[-1, 1]

            plt.plot(u[:, 0], u[:, 1], c=random_rgb, marker='.', markersize=6)
            # start point
            if i == 0:
                plt.text(x_start, y_start, "Start", fontsize=13, color='r')

            # end point
            elif i == len(self.total_saved) - 1:
                plt.text(x_end, y_end, "End", fontsize=13, color='r')

            plt.text(np.mean(u[:, 0]), np.mean(u[:, 1]), "{}.txt".format(str(i + 1)), fontsize=13)

        tm = time.localtime(time.time())
        string = time.strftime('%Y_%m_%d_ %I_%M_%S_%p', tm)
        plt.title(string)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis("equal")
        plt.grid(True)

        plt.savefig(os.path.join(TARGET_SAVE_PATH, f'{string}.png'), dpi=300)
        plt.show()

    def final_process(self):
        # 종료 시 반드시 저장
        self.total_saved.append(self.utm)
        rospy.logwarn(f'Stopped Recording gps path at {self.idx+1}.txt')
        self.save_to_txt()
        self.plot(save_fig=True)
        rospy.loginfo("#### #### PROGRAM END #### ####")


def main():
    mr = MapRecorder()

    rate = rospy.Rate(100)  # 100Hz

    try:
        while not rospy.is_shutdown():
            if mr.exit_event.is_set():
                # exit event 발생 시 최종 프로세스 처리
                mr.final_process()
                break
            
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
        raise e

    finally:
        # restore terminal
        if mr.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, mr.settings)
        plt.close('all')
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)


if __name__ == '__main__':
    main()