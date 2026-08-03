#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray, Header
from morai_msgs.msg import CollisionData as CollisionDataMsg
from morai_msgs.msg import ObjectStatus
from geometry_msgs.msg import Vector3
import time
import sys
import math
import os
import json
from pathlib import Path
import signal

# UDP 통신 라이브러리 임포트
script_dir = Path(__file__).resolve().parent
package_dir = script_dir.parent
sys.path.insert(0, str(package_dir / 'src'))

from lib.network.UDP import Sender, Receiver
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from lib.define.CollisionData import CollisionData as CollisionDataUDP

# JSON 파일에서 파라미터 로드
json_path = str(package_dir)
with open(os.path.join(json_path, "params.json"), 'r') as fp:
    params = json.load(fp)

params = params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]


class MoraiMAINBridge:
    def __init__(self):
        self.node_name = 'morai_main_bridge'
        rospy.init_node(self.node_name, anonymous=False)

        self.vehicle_status_port = params["vehicle_status_dst_port"]

        # IP 및 포트 설정
        self.user_ip = user_ip
        self.host_ip = host_ip
        self.ctrl_cmd_port = params["ctrl_cmd_host_port"]
        self.collision_port = params["collision_data_dst_port"]

        # 내부 상태 초기화
        self.initialize_data()

        rospy.loginfo("%s: Setting up UDP receivers and senders", self.node_name)

        # 선택된 포트로 상태 수신 Receiver
        self.status_receiver = Receiver(self.user_ip, self.vehicle_status_port, EgoVehicleStatus())
        rospy.loginfo("%s: %s:%d VehicleStatus receiver",
                      self.node_name, self.user_ip, self.vehicle_status_port)

        # EgoCtrlCmd 송신
        self.ctrl_sender = Sender(self.host_ip, self.ctrl_cmd_port)
        rospy.loginfo("%s: %s:%d EgoCtrlCmd sender", self.node_name, self.ctrl_cmd_port, self.ctrl_cmd_port)

        # 상태 데이터 수신 대기
        self.is_status = False
        while not rospy.is_shutdown() and not self.is_status:
            status_data = self.status_receiver.get_data()
            if status_data is None:
                rospy.logwarn_throttle(1.0, 'No Status Data. Waiting...')
                time.sleep(0.1)
            else:
                self.is_status = True
                rospy.loginfo("%s: Status data received. Starting main loop.", self.node_name)

        # ROS Pub/Sub
        self.erp_pub = rospy.Publisher('/ERP/serial_data', Float32MultiArray, queue_size=1)
        self.erp_sub = rospy.Subscriber('/Control/serial_data', Float32MultiArray,
                                        self.erp_data_callback, queue_size=1)

        # 100 Hz 타이머
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.main_loop)

        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("%s: Initialization complete", self.node_name)

    def shutdown_hook(self):
        rospy.loginfo("%s: Shutting down...", self.node_name)
        if hasattr(self, 'timer'):
            self.timer.shutdown()
        if hasattr(self, 'status_receiver') and hasattr(self.status_receiver, 'close'):
            self.status_receiver.close()
        if hasattr(self, 'collision_receiver') and hasattr(self, 'close'):
            try:
                self.collision_receiver.close()
            except Exception:
                pass
        if hasattr(self, 'ctrl_sender') and hasattr(self.ctrl_sender, 'close'):
            self.ctrl_sender.close()
        rospy.loginfo("%s: Shutdown complete", self.node_name)

    def initialize_data(self):
        # 수신 상태
        self.control_mode_r_ = 0.0
        self.e_stop_r_ = 0.0
        self.gear_r_ = 0.0
        self.speed_r_ = 0.0
        self.steer_r_ = 0.0
        self.brake_r_ = 0.0
        self.enc_r_ = 0.0

        # 송신 명령
        self.control_mode_t_ = 0x01   # longCmdType (1: throttle mode)
        self.e_stop_t_ = 0x00
        self.gear_t_ = 0
        self.speed_t_ = 0.0           # accel(0..1)
        self.steer_t_ = 0.0           # deg-based scaled value for Morai
        self.brake_t_ = 0.0           # brake(0..1)
        self.alive_t_ = 0

        self.ctrl_cmd_data = EgoCtrlCmd()

    def erp_data_callback(self, msg: Float32MultiArray):
        # Morai longCmdType 1 (accel/brake/steer)
        self.control_mode_t_ = 1
        self.e_stop_t_ = 0

        # gear 매핑 (기존 로직 유지)
        gear_erp = int(msg.data[2])
        if gear_erp == 0:   # N
            self.gear_t_ = 4
        elif gear_erp == 1: # D
            self.gear_t_ = 3
        elif gear_erp == 2: # R
            self.gear_t_ = 2
        else:
            self.gear_t_ = 4

        # accel/brake는 이미 0..1 정규화되어 들어옴
        accel_cmd = float(msg.data[3])  # 0..1
        brake_cmd = float(msg.data[5])  # 0..1

        # 안전 클램프
        accel_cmd = max(0.0, min(1.0, accel_cmd))
        brake_cmd = max(0.0, min(1.0, brake_cmd))

        # 내부 상태 저장
        self.speed_t_ = accel_cmd
        # steer: 내부 파이프라인 스케일 유지 (rad -> deg -> scaled)
        self.steer_t_ = float(msg.data[4]) * (180.0 / 3.14159265358979323846) / 28.17
        self.brake_t_ = brake_cmd
        self.alive_t_ = 0

    def write_ctrl_cmd(self):
        # ctrl_mode/gear 유지
        self.ctrl_cmd_data.ctrl_mode = 2
        self.ctrl_cmd_data.gear = self.gear_t_

        # Throttle 제어 (accel/brake/steer)
        self.ctrl_cmd_data.cmd_type = 1

        # accel/brake는 0..1
        self.ctrl_cmd_data.accel = float(self.speed_t_)
        self.ctrl_cmd_data.brake = float(self.brake_t_)

        # velocity는 throttle 모드에서 미사용
        self.ctrl_cmd_data.velocity = 0.0

        # steer는 스케일된 값 사용
        self.ctrl_cmd_data.steer = float(self.steer_t_)

        # 송신
        self.ctrl_sender.send(self.ctrl_cmd_data)
        rospy.loginfo(
            "Send Ctrl: mode=%d, gear=%d, cmd_type=%d, accel=%.3f, brake=%.3f, steer=%.3f",
            self.ctrl_cmd_data.ctrl_mode,
            self.ctrl_cmd_data.gear,
            self.ctrl_cmd_data.cmd_type,
            self.ctrl_cmd_data.accel,
            self.ctrl_cmd_data.brake,
            self.ctrl_cmd_data.steer
        )

    def read_vehicle_status(self):
        status = self.status_receiver.get_data()
        rospy.loginfo("Checking for vehicle status data..." + str(status))
        if status is None:
            return

        self.control_mode_r_ = float(status.ctrl_mode)
        self.e_stop_r_ = 0.0
        self.gear_r_ = float(status.gear)
        self.speed_r_ = status.signed_vel  # km/h or m/s depends on SDK; original divides by 3.6 below
        self.steer_r_ = status.steer
        self.brake_r_ = status.brake
        self.enc_r_ = 0.0

        msg = Float32MultiArray()
        msg.data = [
            self.control_mode_r_,
            self.e_stop_r_,
            self.gear_r_,
            self.speed_r_ / 3.6,                 # to m/s
            self.steer_r_ * (math.pi / 180.0),   # to rad
            self.brake_r_,
            self.enc_r_
        ]
        self.erp_pub.publish(msg)
        rospy.loginfo("Recv Statusssssss: mode=%.0f, gear=%.0f, vel=%.2f m/s, steer=%.3f rad, brake=%.3f",
                      msg.data[0], msg.data[2], msg.data[3], msg.data[4], msg.data[5])

    def main_loop(self, event):
        self.write_ctrl_cmd()
        self.read_vehicle_status()


def signal_handler(sig, frame):
    rospy.loginfo("Ctrl+C pressed. Shutting down...")
    rospy.signal_shutdown("User requested shutdown")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)

    try:
        bridge = MoraiMAINBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt")
    except Exception as e:
        rospy.logerr("Unexpected error: %s", str(e))
    finally:
        rospy.loginfo("Node terminated")


if __name__ == '__main__':
    main()