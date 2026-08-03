#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import math
import json
import os
import sys
import rospy
from pathlib import Path
from sensor_msgs.msg import NavSatFix

# JSON 파일에서 파라미터 로드
script_dir = Path(__file__).resolve().parent
package_dir = script_dir.parent

json_path = str(package_dir)
with open(os.path.join(json_path, "config.json"), 'r') as fp:
    params = json.load(fp)

params = params["params"]
host_ip = params["host_ip"]
gps_dst_port = params["gps_dst_port"]


def dm_to_decimal(dm_value):
    # Convert NMEA degrees+minutes (ddmm.mmmm) to decimal degrees
    degrees = int(dm_value / 100)
    minutes = dm_value - degrees * 100
    return degrees + (minutes / 60.0)


def main():
    rospy.init_node('gps_publisher', anonymous=False)

    rospy.loginfo('GPS Publisher - Binding to {}:{}'.format(host_ip, gps_dst_port))

    pub = rospy.Publisher('fix', NavSatFix, queue_size=10)

    # Set up UDP socket once
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = (host_ip, gps_dst_port)
    sock.bind(server_address)
    sock.settimeout(1.0)  # allow clean shutdown

    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(8080)
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logwarn_throttle(5.0, 'Socket error: {}'.format(e))
            continue

        if not data:
            continue

        try:
            gps_sentence = data.decode('utf-8', errors='ignore').strip()
        except Exception:
            continue

        if not gps_sentence.startswith('$GPRMC'):
            continue

        fields = gps_sentence.split(',')

        # Basic field presence check
        if len(fields) < 7:
            continue

        # fields[2] = Status 'A' valid, 'V' void
        if fields[2] != 'A':
            continue

        try:
            lat_val = float(fields[3])        # ddmm.mmmm
            lat_hemi = fields[4]              # 'N' or 'S'
            lon_val = float(fields[5])        # dddmm.mmmm
            lon_hemi = fields[6]              # 'E' or 'W'
        except Exception:
            continue

        # Convert to decimal degrees
        latitude = dm_to_decimal(lat_val)
        longitude = dm_to_decimal(lon_val)

        if lat_hemi == 'S':
            latitude = -latitude
        if lon_hemi == 'W':
            longitude = -longitude

        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'gps'
        msg.latitude = latitude
        msg.longitude = longitude
        # altitude may be unknown in GPRMC
        msg.altitude = float('nan')

        pub.publish(msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass