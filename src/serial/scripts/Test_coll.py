#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket, struct, time, math

IP = "127.0.0.1"
PORT = 9092              # 수신 포트
MAX_HDR_SCAN = 64        # 헤더 후보 범위
MAX_OBJS = 5

def sane(v, limit=1e7):
    try:
        return math.isfinite(v) and abs(v) < limit
    except:
        return False

def score_objects(objs):
    sc = 0
    for t, oid, px, py, pz, gox, goy, goz in objs:
        if t in (-1,0,1,2): sc += 2
        if sane(px) and sane(py) and sane(pz): sc += 2
        if sane(gox, 1e9) and sane(goy, 1e9) and sane(goz, 1e6): sc += 2
        if -32768 <= oid <= 32767: sc += 1
    return sc

def try_parse(buf, base, endian):
    ts_fmt = endian + "Q"
    obj_fmt = endian + "hh" + "f"*6
    obj_size = struct.calcsize(obj_fmt)
    if len(buf) < base + 8 + obj_size:
        return None
    ts = struct.unpack_from(ts_fmt, buf, base)[0]
    payload = buf[base+8:]
    n = min(MAX_OBJS, len(payload)//obj_size)
    objs = []
    for i in range(n):
        off = i*obj_size
        if off + obj_size > len(payload): break
        t, oid, px, py, pz, gox, goy, goz = struct.unpack_from(obj_fmt, payload, off)
        objs.append((t, oid, px, py, pz, gox, goy, goz))
    return ts, objs

def auto_parse(buf):
    best = None
    for endian in ("<", ">"):
        for base in range(0, MAX_HDR_SCAN+1):
            out = try_parse(buf, base, endian)
            if not out: continue
            ts, objs = out
            if len(objs) == 0 or len(objs) > MAX_OBJS: continue
            sc = score_objects(objs)
            if all(o[0] not in (-1,0,1,2) for o in objs): 
                continue
            if (best is None) or (sc > best[0]):
                best = (sc, endian, base, ts, objs)
    return best

def is_nonzero_obj(obj):
    """idx1이 '들어왔다'고 볼 유효값 판정: id가 0이 아니거나, 위치/오프셋 합이 유의미"""
    t, oid, px, py, pz, gox, goy, goz = obj
    if oid != 0: 
        return True
    if abs(px) + abs(py) + abs(pz) > 1e-6:
        return True
    if abs(gox) + abs(goy) + abs(goz) > 1e-6:
        return True
    return False

def fmt_row(i, rec, suffix=""):
    t, oid, px, py, pz, gox, goy, goz = rec
    name = { -1:"Ego", 0:"Ped", 1:"Veh", 2:"Obj" }.get(t, f"?{t}")
    return (f"{i:>4} | {t:>4} | {name:<5} | {oid:>3} | "
            f"{px:>11.6f} | {py:>11.6f} | {pz:>11.6f} | "
            f"{gox:>8.3f} | {goy:>8.3f} | {goz:>8.3f}{suffix}")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, PORT))
    print(f"[listening] udp://{IP}:{PORT} (auto-detect header offset & endianness)")

    locked_idx1 = None   # ✅ 처음 들어온 idx1 값 고정 저장
    locked_meta  = ""    # 정보 표시용(when locked)

    while True:
        buf, addr = sock.recvfrom(2048)
        now = time.strftime("%H:%M:%S")
        res = auto_parse(buf)
        if not res:
            print(f"\n[{now}] from {addr}, bytes={len(buf)} -> could not auto-parse")
            continue

        sc, endian, base, ts, objs = res
        ed = "LE" if endian=="<" else "BE"

        # idx0(ego) 현재값
        ego = objs[0] if len(objs) >= 1 else None
        # idx1 현재 프레임 값(있을 수도/없을 수도)
        cur1 = objs[1] if len(objs) >= 2 else None

        # ✅ 아직 잠기지 않았고, 이번 프레임의 idx1이 '유효한 비-제로'이면 그 값을 잠금
        if locked_idx1 is None and cur1 is not None and is_nonzero_obj(cur1):
            locked_idx1 = cur1
            locked_meta  = f"[LOCKED at {now} | base={base}B | endian={ed} | ts={ts}]"
            print("\n--- FIRST idx1 COLLISION LOCKED ---", locked_meta)

        print(f"\n[{now}] from {addr}, bytes={len(buf)} | base={base}B | endian={ed} | ts={ts} | objs={len(objs)} | score={sc}")
        print(" idx | type | name  |  id |        pos_x |        pos_y |        pos_z |   goff_x |   goff_y |   goff_z ")
        print("-----+------+-------+-----+-------------+-------------+-------------+----------+----------+----------")

        # idx0: 항상 현재값 출력
        if ego is not None:
            print(fmt_row(0, ego))

        # idx1: 잠금값이 있으면 그 값만 계속 출력, 없으면 현재 프레임 값 출력(참고용)
        if locked_idx1 is not None:
            print(fmt_row(1, locked_idx1, "   [locked]"))
        elif cur1 is not None:
            print(fmt_row(1, cur1, "   [waiting lock]"))

        # 나머지 인덱스는 필요하면 계속 출력(원본과 동일하게 유지하려면 주석 해제)
        # for i in range(2, len(objs)):
        #     print(fmt_row(i, objs[i]))

if __name__ == "__main__":
    main()
