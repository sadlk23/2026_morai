#!/usr/bin/env python3
import socket, struct, json, os, sys, rospy
from pathlib import Path
from std_msgs.msg import Header
from morai.msg import CollisionData, ObjectStatus

# JSON 파일에서 파라미터 로드
script_dir = Path(__file__).resolve().parent
package_dir = script_dir.parent

json_path = str(package_dir)
with open(os.path.join(json_path, "config.json"), 'r') as fp:
    params = json.load(fp)

params = params["params"]
host_ip = params["host_ip"]
collision_data_dst_port = params["collision_data_dst_port"]


def parse_packet(data, endian="<"):
    # 기대 길이 체크(181 bytes)
    if len(data) < 181:
        return None
    # 1) 패킷 선두 timestamp(double, 8B)
    ts = struct.unpack(endian + "d", data[0:8])[0]

    # 2) 오브젝트 5개 슬롯(each 28B = hh + 6f)
    objs, offset = [], 8
    obj_fmt = endian + "hhffffff"   # type,id, pos(x,y,z), g_off(x,y,z)
    obj_size = struct.calcsize(obj_fmt)

    for i in range(5):
        s, e = offset + i*obj_size, offset + (i+1)*obj_size
        if e > len(data):
            break
        t, oid, x, y, z, gx, gy, gz = struct.unpack(obj_fmt, data[s:e])

        # 완전 빈 슬롯(전부 0) 스킵하고 싶으면 주석 해제
        # if t == 0 and oid == 0 and x == 0.0 and y == 0.0 and z == 0.0 and gx == 0.0 and gy == 0.0 and gz == 0.0:
        #     continue

        os = ObjectStatus()
        os.obj_type, os.obj_id = t, oid
        os.x, os.y, os.z = x, y, z
        os.global_offset_x, os.global_offset_y, os.global_offset_z = gx, gy, gz
        objs.append(os)

    # 전역 오프셋(대표값): 첫 오브젝트 기준
    goff = (objs[0].global_offset_x, objs[0].global_offset_y, objs[0].global_offset_z) if objs else (0.0,0.0,0.0)
    return ts, objs, goff


def build_msg(ts, objs, goff):
    msg = CollisionData()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "CollisionData"
    msg.timestamp = float(ts)
    msg.global_offset_x, msg.global_offset_y, msg.global_offset_z = map(float, goff)
    msg.collision_object = objs
    msg.collision_objecta = []
    return msg


def main():
    rospy.init_node("morai_collision_udp_node", anonymous=False)

    # JSON에서 IP와 포트 읽기 (기본값으로 rospy.get_param도 지원)
    bind_ip = rospy.get_param("~bind_ip", host_ip)
    bind_port = int(rospy.get_param("~bind_port", collision_data_dst_port))
    endian = rospy.get_param("~endian", "<")
    topic_out = rospy.get_param("~topic", "/morai/collision")

    # event-style: 충돌 중일 때만 퍼블리시
    pub = rospy.Publisher(topic_out, CollisionData, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, bind_port))
    sock.settimeout(1.0)

    rospy.loginfo("UDP %s:%d → event publish: %s", bind_ip, bind_port, topic_out)

    in_collision = False  # 현재 프레임이 '충돌 중'인지 상태
    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Socket error: %s", e)
            continue

        parsed = parse_packet(data, endian=endian)
        if not parsed:
            rospy.logwarn_throttle(5.0, "Invalid packet size: %d", len(data))
            continue

        ts, objs, goff = parsed
        has_collision = len(objs) > 0

        if has_collision:
            # 충돌이 시작됐거나 계속되는 동안에는 매 프레임 퍼블리시
            if not in_collision:
                rospy.loginfo("Collision START (%d object(s))", len(objs))
            msg = build_msg(ts, objs, goff)
            pub.publish(msg)
        else:
            # 방금 충돌이 끝났다면 한 번만 로그
            if in_collision:
                rospy.loginfo("Collision END")

        in_collision = has_collision


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass