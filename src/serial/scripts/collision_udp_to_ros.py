#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket, struct, time, math, select
import rospy
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import Vector3
from morai_msgs.msg import CollisionData as CollisionDataMsg
from morai_msgs.msg import ObjectStatus

# -----------------------
# 유틸
# -----------------------
def sane(v, limit=1e7):
    try:
        return math.isfinite(v) and abs(v) < limit
    except Exception:
        return False

def score_objects(objs):
    sc = 0
    for t, oid, px, py, pz, gox, goy, goz in objs:
        if t in (-1, 0, 1, 2): sc += 2
        if sane(px) and sane(py) and sane(pz): sc += 2
        if sane(gox, 1e9) and sane(goy, 1e9) and sane(goz, 1e6): sc += 2
        if -32768 <= oid <= 32767: sc += 1
    return sc

def try_parse(buf, base, endian):
    ts_fmt = endian + "Q"                 # 8B timestamp
    obj_fmt = endian + "hh" + "f"*6       # 28B object
    obj_size = struct.calcsize(obj_fmt)
    if len(buf) < base + 8 + obj_size:
        return None
    ts = struct.unpack_from(ts_fmt, buf, base)[0]
    payload = buf[base + 8:]
    n = min(5, len(payload) // obj_size)
    objs = []
    for i in range(n):
        off = i * obj_size
        if off + obj_size > len(payload): break
        t, oid, px, py, pz, gox, goy, goz = struct.unpack_from(obj_fmt, payload, off)
        objs.append((t, oid, px, py, pz, gox, goy, goz))
    return ts, objs

def auto_detect_base_endian(buf, max_hdr_scan=64):
    best = None
    for endian in ("<", ">"):
        for base in range(0, max_hdr_scan + 1):
            out = try_parse(buf, base, endian)
            if not out: continue
            ts, objs = out
            if len(objs) == 0 or len(objs) > 5: continue
            if all(o[0] not in (-1, 0, 1, 2) for o in objs): continue
            sc = score_objects(objs)
            if (best is None) or (sc > best[0]):
                best = (sc, endian, base, ts, objs)
    return best  # (score, endian, base, ts, objs)

def is_nonzero_obj(obj):
    """idx1이 '들어왔다'라고 볼 기준: id!=0 또는 pos/offset 합이 유의미"""
    t, oid, px, py, pz, gox, goy, goz = obj
    if oid != 0: return True
    if abs(px) + abs(py) + abs(pz) > 1e-6: return True
    if abs(gox) + abs(goy) + abs(goz) > 1e-6: return True
    return False

def type_to_name(t: int) -> str:
    if t == -1: return "Ego"
    if t == 0:  return "Person"
    if t == 1:  return "Vehicle"
    if t == 2:  return "Object"
    return f"Unknown_{t}"

def make_object_status(t, oid, px, py, pz, gox, goy, goz, ego_name="2023_Hyundai_Ioniq5"):
    obj = ObjectStatus()
    obj.unique_id = int(oid)
    obj.type = int(t)
    if obj.type == -1: obj.name = ego_name
    elif obj.type == 0: obj.name = "Person"
    elif obj.type == 1: obj.name = "Vehicle"
    elif obj.type == 2: obj.name = "Object"
    else: obj.name = f"Unknown_{obj.type}"
    obj.heading = 0.0
    obj.position = Vector3(float(px), float(py), float(pz))
    obj.velocity = Vector3(0.0, 0.0, 0.0)
    obj.acceleration = Vector3(0.0, 0.0, 0.0)
    obj.size = Vector3(0.0, 0.0, 0.0)
    return obj

# -----------------------
# 메인 노드
# -----------------------
class CollisionUDP2ROS(object):
    def __init__(self):
        self.ip = rospy.get_param("~ip", "127.0.0.1")  # IP 지정
        self.port = int(rospy.get_param("~port", 9092)) # Port 번호 지정
        self.max_hdr_scan = int(rospy.get_param("~max_hdr_scan", 64))
        self.publish_hz = float(rospy.get_param("~publish_hz", 50.0))
        self.lock_duration = float(rospy.get_param("~lock_duration", 3.0))  # 유지 시간(초)
        self.frame_id = rospy.get_param("~frame_id", "CollisionData")
        self.ego_name = rospy.get_param("~ego_name", "2023_Hyundai_Ioniq5")

        # Publishers
        self.pub = rospy.Publisher("/Ioniq5/collisionData", CollisionDataMsg, queue_size=10)
        self.flag_pub = rospy.Publisher("/Ioniq5/collisionData_flag", Int32, queue_size=10)  # 0/1 상태 토픽

        # 소켓 준비 (논블로킹)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.setblocking(False)
        rospy.loginfo("Listening UDP %s:%d", self.ip, self.port)

        # 자동탐지 결과 고정
        self._base = None
        self._endian = None
        self._obj_fmt = None
        self._obj_size = None

        # 상태 저장: ego 최신, idx1 locked(+시각)
        self.last_ego = None                   # (t,oid,px,py,pz,gox,goy,goz)
        self.locked_idx1 = None                # (t,oid,px,py,pz,gox,goy,goz)
        self.locked_ts = None                  # 패킷 내 timestamp(raw)
        self.locked_at_ros = None              # ROS 시간(잠근 시각)

        # global offset 유지
        self.global_offset = (0.0, 0.0, 0.0)

        # 상태 플래그 / 콘솔 출력 제어
        self.last_flag = 0   # 직전 퍼블리시한 0/1 값

        # ✅ LOCKED 로그를 Boom!!! 뒤에 찍기 위해 잠시 저장하는 버퍼
        self._pending_lock_log = None  # tuple: (duration, oid, t, name, px, py, pz)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-3, self.publish_hz)), self._on_timer)

    def _parse_with_fixed(self, buf):
        ts_fmt = self._endian + "Q"
        ts = struct.unpack_from(ts_fmt, buf, self._base)[0]
        payload = buf[self._base + 8:]
        objs = []
        for i in range(min(5, len(payload) // self._obj_size)):
            off = i * self._obj_size
            t, oid, px, py, pz, gox, goy, goz = struct.unpack_from(self._obj_fmt, payload, off)
            objs.append((t, oid, px, py, pz, gox, goy, goz))
        return ts, objs

    def _receive_once(self):
        """소켓에서 1회 수신 (있으면 처리)"""
        r, _, _ = select.select([self.sock], [], [], 0.0)
        if not r:
            return
        buf, addr = self.sock.recvfrom(2048)
        if len(buf) < 64:
            return

        # 첫 패킷에서 base/엔디안 탐지
        if self._base is None or self._endian is None:
            detect = auto_detect_base_endian(buf, self.max_hdr_scan)
            if not detect: return
            sc, endian, base, ts, objs = detect
            self._base = base
            self._endian = endian
            self._obj_fmt = self._endian + "hh" + "f"*6
            self._obj_size = struct.calcsize(self._obj_fmt)
            rospy.loginfo("Detected base=%dB endian=%s (score=%d, objs=%d)",
                          self._base, "LE" if self._endian=="<" else "BE", sc, len(objs))

        # 고정된 파서로 파싱
        ts, objs = self._parse_with_fixed(buf)
        if not objs:
            return

        # idx0(ego) 최신 업데이트
        ego = objs[0]
        self.last_ego = ego

        # global offset 업데이트 (ego에서)
        _, _, _, _, _, gox, goy, goz = ego
        if any([abs(gox) > 1e-6, abs(goy) > 1e-6, abs(goz) > 1e-6]):
            self.global_offset = (gox, goy, goz)

        # idx1(충돌 대상) 고정(처음 유효값 들어오면 잠금)
        if len(objs) >= 2:
            cand1 = objs[1]
            if self.locked_idx1 is None and is_nonzero_obj(cand1):
                self.locked_idx1 = cand1
                self.locked_ts = ts
                self.locked_at_ros = rospy.Time.now()   # 잠근 시각 기록

                # 🔸 여기서는 즉시 로그를 찍지 않고 pending으로 저장만 한다
                t, oid, px, py, pz, _, _, _ = cand1
                name = type_to_name(int(t))
                self._pending_lock_log = (self.lock_duration, int(oid), int(t), name, float(px), float(py), float(pz))

    def _on_timer(self, event):
        # 수신 처리
        self._receive_once()

        # 잠금 유지 시간 경과 시 자동 초기화
        if self.locked_idx1 is not None and self.locked_at_ros is not None:
            if (rospy.Time.now() - self.locked_at_ros).to_sec() >= self.lock_duration:
                rospy.loginfo("UNLOCK (timeout %.1fs). Waiting for next collision...", self.lock_duration)
                self.locked_idx1 = None
                self.locked_ts = None
                self.locked_at_ros = None

        # === 상태 플래그 계산 (1: 잠금 유지 중 == 충돌 상태, 0: 정상) ===
        flag = 1 if self.locked_idx1 is not None else 0

        # 콘솔 출력 (상태 변화시에만)
        if flag != self.last_flag:
            if flag == 1:
                # ✅ 먼저 Boom!!!
                rospy.loginfo("Boom!!!") # 충돌 발생
                # ✅ 이어서 LOCKED 로그 (pending이 있으면)
                if self._pending_lock_log is not None:
                    dur, oid, t, name, px, py, pz = self._pending_lock_log
                    rospy.loginfo("LOCKED for %.1fs: id=%d type=%d name=%s pos=(%.3f,%.3f,%.3f)",
                                  dur, oid, t, name, px, py, pz)
                    self._pending_lock_log = None
            else:
                # ✅ UNLOCK 이후에 Go!!! (UNLOCK 로그는 위에서 이미 출력됨)
                rospy.loginfo("Go!!!") # 충돌 후 3초 경과 정상 상태 복귀
            self.last_flag = flag

        # 상태 플래그 퍼블리시
        self.flag_pub.publish(Int32(data=flag))

        # 상세 메시지 퍼블리시 (/Ioniq5/collision)
        msg = CollisionDataMsg()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.global_offset_x, msg.global_offset_y, msg.global_offset_z = self.global_offset
        msg.collision_object = []

        # idx0: 항상 최신
        if self.last_ego is not None:
            t, oid, px, py, pz, gox, goy, goz = self.last_ego
            msg.collision_object.append(
                make_object_status(t, oid, px, py, pz, gox, goy, goz, ego_name=self.ego_name)
            )

        # idx1: 잠금 유지 중일 때만 고정 값 출력
        if self.locked_idx1 is not None:
            t, oid, px, py, pz, gox, goy, goz = self.locked_idx1
            msg.collision_object.append(
                make_object_status(t, oid, px, py, pz, gox, goy, goz, ego_name=self.ego_name)
            )

        self.pub.publish(msg)

def main():
    rospy.init_node("collision_udp_to_ros", anonymous=False)
    node = CollisionUDP2ROS()
    rospy.loginfo("collision_udp_to_ros started.")
    rospy.spin()

if __name__ == "__main__":
    main()
