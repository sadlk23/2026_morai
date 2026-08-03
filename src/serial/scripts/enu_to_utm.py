#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped
import pyproj
import math


class ENUtoUTMPublisher(object):
    def __init__(self):
        # ENU 원점의 위경도 좌표 (파라미터로 설정 가능)
        self.origin_lat = rospy.get_param('~origin_latitude', 37.240474999999996)
        self.origin_lon = rospy.get_param('~origin_longitude', 126.77332833333334)
        self.utm_zone = rospy.get_param('~utm_zone', 52)
        
        # Publisher & Subscriber
        self.publisher = rospy.Publisher('/Local/utm', PointStamped, queue_size=1)
        self.subscription = rospy.Subscriber('/fix', PointStamped, self.enu_callback, queue_size=1)
        
        # WGS84 -> UTM 52N 변환기
        # 대한민국 경기도, 충청도 지역 UTM 52N (WGS84 / UTM zone 52N: EPSG:32652)
        self.transformer = pyproj.Transformer.from_crs(
            "EPSG:4326", 
            f"EPSG:326{self.utm_zone}", 
            always_xy=True
        )
        
        # 원점의 UTM 좌표 계산
        self.origin_utm_easting, self.origin_utm_northing = self.transformer.transform(
            self.origin_lon, self.origin_lat
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("ENU to UTM Publisher Node Started")
        rospy.loginfo(f"Origin Latitude:  {self.origin_lat:.10f}")
        rospy.loginfo(f"Origin Longitude: {self.origin_lon:.10f}")
        rospy.loginfo(f"Origin UTM Easting:  {self.origin_utm_easting:.2f} m")
        rospy.loginfo(f"Origin UTM Northing: {self.origin_utm_northing:.2f} m")
        rospy.loginfo(f"UTM Zone: {self.utm_zone}N")
        rospy.loginfo("="*60)

    def enu_callback(self, msg):
        """
        ENU 좌표를 받아서 UTM 좌표로 변환
        
        msg.point.x: ENU X (East, 동쪽 방향, meters)
        msg.point.y: ENU Y (North, 북쪽 방향, meters)
        msg.point.z: ENU Z (Up, 위쪽 방향, meters)
        """
        enu_x = msg.point.x
        enu_y = msg.point.y
        enu_z = msg.point.z
        
        # 유효성 체크 (NaN 방지)
        if any(math.isnan(v) for v in [enu_x, enu_y, enu_z]):
            rospy.logwarn("Received NaN values in ENU coordinates. Skipping...")
            return
        
        # ENU -> UTM 변환
        # UTM = Origin_UTM + ENU_offset
        utm_easting = self.origin_utm_easting + enu_x
        utm_northing = self.origin_utm_northing + enu_y
        utm_altitude = enu_z
        
        # UTM 좌표 메시지 생성
        utm_point = PointStamped()
        utm_point.header.stamp = rospy.Time.now()
        utm_point.header.frame_id = 'map'
        utm_point.point.x = utm_easting
        utm_point.point.y = utm_northing
        utm_point.point.z = utm_altitude
        
        # 발행
        self.publisher.publish(utm_point)
        
        rospy.loginfo_throttle(1.0, 
            f"ENU: ({enu_x:.2f}, {enu_y:.2f}, {enu_z:.2f}) -> "
            f"UTM: ({utm_easting:.2f}, {utm_northing:.2f}, {utm_altitude:.2f})")


def main():
    rospy.init_node('enu_to_utm', anonymous=False)
    node = ENUtoUTMPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()