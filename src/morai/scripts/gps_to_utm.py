#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import pyproj
import math


class OdomPublisher(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/Local/utm', PointStamped, queue_size=1)
        self.subscription = rospy.Subscriber('/fix', NavSatFix, self.gps_callback, queue_size=1)
        # 대한민국 경기도, 충청도 지역 UTM 52N (WGS84 / UTM zone 52N: EPSG:32652)
        # 참고: 남반구면 EPSG:32752 (52S)
        self.transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        # 유효성 체크 (NaN 방지)
        if any(math.isnan(v) for v in [lat, lon]):
            return

        # 위경도 -> UTM
        easting, northing = self.transformer.transform(lon, lat)

        utm_point = PointStamped()
        utm_point.header.stamp = rospy.Time.now()
        utm_point.header.frame_id = 'map'
        utm_point.point.x = easting
        utm_point.point.y = northing
        utm_point.point.z = msg.altitude

        self.publisher.publish(utm_point)


def main():
    rospy.init_node('utm', anonymous=False)
    node = OdomPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()