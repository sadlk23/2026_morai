#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import math
import json
import os
import sys
import rospy
from pathlib import Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# JSON 파일에서 파라미터 로드
script_dir = Path(__file__).resolve().parent
package_dir = script_dir.parent

json_path = str(package_dir)
with open(os.path.join(json_path, "config.json"), 'r') as fp:
    params = json.load(fp)

params = params["params"]
host_ip = params["host_ip"]
imu_dst_port = params["imu_dst_port"]


def quaternion_to_euler(q0, q1, q2, q3):
    # roll (x-axis rotation)
    t0 = +2.0 * (q0 * q1 + q2 * q3)
    t1 = +1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (q0 * q2 - q3 * q1)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (q0 * q3 + q1 * q2)
    t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def main():
    rospy.init_node('imu_publisher', anonymous=False)

    rospy.loginfo('IMU Publisher - Binding to {}:{}'.format(host_ip, imu_dst_port))

    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    heading_pub = rospy.Publisher('/Local/heading', Float64, queue_size=10)

    # Set up UDP socket once
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = (host_ip, imu_dst_port)
    sock.bind(server_address)
    sock.settimeout(1.0)

    rate = rospy.Rate(100)  # 0.01s
    parsed_data = []

    while not rospy.is_shutdown():
        try:
            raw_data, _ = sock.recvfrom(65535)
        except socket.timeout:
            rate.sleep()
            continue
        except Exception as e:
            rospy.logwarn_throttle(5.0, 'Socket error: {}'.format(e))
            rate.sleep()
            continue

        if len(raw_data) < 105:
            continue

        try:
            header = raw_data[0:9].decode(errors='ignore')
        except Exception:
            continue

        if header != '#IMUData$':
            continue

        try:
            # data_length (int) is present but unused here
            _ = struct.unpack('i', raw_data[9:13])
            # skip reserved bytes [13:25] as in original layout
            imu_data = struct.unpack('10d', raw_data[25:105])
            parsed_data = imu_data
        except Exception as e:
            rospy.logwarn_throttle(5.0, 'Parse error: {}'.format(e))
            continue

        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'imu'

        msg.orientation.w = round(parsed_data[1], 2)
        msg.orientation.x = round(parsed_data[2], 2)
        msg.orientation.y = round(parsed_data[3], 2)
        msg.orientation.z = round(parsed_data[4], 2)

        msg.angular_velocity.x = round(parsed_data[5], 2)
        msg.angular_velocity.y = round(parsed_data[6], 2)
        msg.angular_velocity.z = round(parsed_data[7], 2)

        msg.linear_acceleration.x = round(parsed_data[8], 2)
        msg.linear_acceleration.y = round(parsed_data[9], 2)
        # msg.linear_acceleration.z is unknown in your payload

        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )

        heading_msg = Float64()
        heading_msg.data = yaw  # radians

        heading_pub.publish(heading_msg)
        imu_pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass