#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import socket
import struct
import time
import argparse

"""
dual_cam_send.py
- เปิดกล้องหลายตัว (เช่น [0,1]) ที่ 640x480@30
- ย่อภาพด้วยการหาร resize_div (เช่น /3) ก่อนเข้ารหัส JPEG
- ส่งผ่าน TCP: [1 byte cam_id][8 bytes size(Big-Endian)][JPEG bytes]
- ไม่มี ROS
"""

def connect_with_retry(server_ip, server_port, retry_delay=3.0):
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((server_ip, server_port))
            print(f"[OK] Connected to {server_ip}:{server_port}")
            return sock
        except (socket.error, ConnectionRefusedError) as e:
            print(f"[WARN] Connect failed: {e}. Retry in {retry_delay:.0f}s...")
            time.sleep(retry_delay)

def open_cameras(cam_indices, width, height, fps):
    caps = []
    for idx in cam_indices:
        # บางเครื่อง Windows อาจต้องใช้ cv2.CAP_DSHOW -> cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        cap = cv2.VideoCapture(idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS,          fps)
        if not cap.isOpened():
            print(f"[WARN] Camera index {idx} open failed.")
        caps.append(cap)
    return caps

def send_frame(sock, cam_id, frame_bgr, out_w, out_h, jpeg_q):
    # resize
    resized = cv2.resize(frame_bgr, (out_w, out_h), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
    if not ok:
        print(f"[WARN] JPEG encode failed for cam {cam_id}")
        return False
    data = encoded.tobytes()
    # ใช้ big-endian เพื่อความชัดเจนข้ามแพลตฟอร์ม
    header = struct.pack('!BQ', cam_id & 0xFF, len(data))  # B=1 byte, Q=8 bytes, big-endian
    sock.sendall(header + data)
    return True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server_ip",   type=str, default="192.168.50.177")
    ap.add_argument("--server_port", type=int, default=8080)
    ap.add_argument("--cam_indices", type=int, nargs="+", default=[0, 2])
    ap.add_argument("--width",  type=int, default=1080)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps",    type=int, default=30)
    ap.add_argument("--jpeg_quality", type=int, default=80)
    ap.add_argument("--resize_div",   type=int, default=2)
    args = ap.parse_args()

    out_w = max(1, args.width  // max(1, args.resize_div))
    out_h = max(1, args.height // max(1, args.resize_div))

    print(f"[INFO] Start sender → cams={args.cam_indices}, "
          f"{args.width}x{args.height}@{args.fps} → resize /{args.resize_div} "
          f"({out_w}x{out_h}), JPEG q={args.jpeg_quality}, "
          f"send to {args.server_ip}:{args.server_port}")

    caps = open_cameras(args.cam_indices, args.width, args.height, args.fps)
    sock = connect_with_retry(args.server_ip, args.server_port)

    period = 1.0 / max(1, args.fps)
    try:
        while True:
            t0 = time.time()
            for cam_i, cap in zip(args.cam_indices, caps):
                if not cap or not cap.isOpened():
                    continue
                ret, frame = cap.read()
                if not ret or frame is None:
                    print(f"[WARN] Failed to read frame from cam {cam_i}")
                    continue
                try:
                    ok = send_frame(sock, cam_i, frame, out_w, out_h, args.jpeg_quality)
                    if not ok:
                        print(f"[WARN] Send failed for cam {cam_i}")
                except (socket.error, BrokenPipeError) as e:
                    print(f"[WARN] Send failed (cam {cam_i}): {e}. Reconnecting...")
                    try:
                        sock.close()
                    except Exception:
                        pass
                    sock = connect_with_retry(args.server_ip, args.server_port)
            # คุมอัตราเฟรมโดยคร่าว ๆ
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass
        for c in caps:
            try:
                if c:
                    c.release()
            except Exception:
                pass
        print("[INFO] Sender stopped.")

if __name__ == "__main__":
    main()
