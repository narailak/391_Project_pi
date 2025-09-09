#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import socket
import struct
import time
import rclpy
from rclpy.node import Node

"""
dual_cam_send_node
- เปิดกล้องหลายตัว (เช่น [0,1]) ที่ 640x480@30
- ย่อภาพด้วยการหาร 3 ก่อนเข้ารหัส JPEG
- ส่งผ่าน TCP: [1 byte cam_id][8 bytes size][JPEG bytes]
- ไม่ publish/subscribe topic (แม้เป็น ROS node)
"""

class DualCamSendNode(Node):
    def __init__(self):
        super().__init__('dual_cam_send_node')

        # ---- ROS params ----
        self.declare_parameter('server_ip', '10.223.62.4')
        self.declare_parameter('server_port', 8080)
        self.declare_parameter('cam_indices', [0, 1])
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('resize_div', 3)

        self.server_ip   = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = int(self.get_parameter('server_port').get_parameter_value().integer_value)
        self.cam_indices = list(self.get_parameter('cam_indices').get_parameter_value().integer_array_value)
        self.width       = int(self.get_parameter('width').get_parameter_value().integer_value)
        self.height      = int(self.get_parameter('height').get_parameter_value().integer_value)
        self.fps         = int(self.get_parameter('fps').get_parameter_value().integer_value)
        self.jpeg_q      = int(self.get_parameter('jpeg_quality').get_parameter_value().integer_value)
        self.resize_div  = int(self.get_parameter('resize_div').get_parameter_value().integer_value) or 3

        self.out_w = max(1, self.width  // self.resize_div)
        self.out_h = max(1, self.height // self.resize_div)

        # ---- เปิดกล้อง ----
        self.caps = []
        for idx in self.cam_indices:
            cap = cv2.VideoCapture(idx)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS,          self.fps)
            if not cap.isOpened():
                self.get_logger().warn(f'Camera index {idx} open failed.')
            self.caps.append(cap)

        # ---- เชื่อมต่อ TCP ----
        self.sock = None
        self._connect()

        # ---- Timer loop ตาม fps ตั้งต้น (จะอ่าน+ส่งทุกรอบ) ----
        period = 1.0 / max(1, self.fps)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'Start dual_cam_send_node → cams={self.cam_indices}, '
            f'{self.width}x{self.height}@{self.fps} → resize /{self.resize_div} '
            f'({self.out_w}x{self.out_h}), JPEG q={self.jpeg_q}, '
            f'send to {self.server_ip}:{self.server_port}'
        )

    def _connect(self):
        while rclpy.ok():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.server_ip, self.server_port))
                self.get_logger().info('Connected to server.')
                return
            except (socket.error, ConnectionRefusedError) as e:
                self.get_logger().warn(f'Connect failed: {e}. Retry in 3s...')
                time.sleep(3)

    def _send_frame(self, cam_id: int, frame_bgr):
        # resize ก่อนส่ง
        resized = cv2.resize(frame_bgr, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
        if not ok:
            self.get_logger().warn(f'JPEG encode failed for cam {cam_id}')
            return
        data = encoded.tobytes()
        header = struct.pack('BQ', cam_id & 0xFF, len(data))  # 1 ไบต์ cam_id + 8 ไบต์ size

        try:
            self.sock.sendall(header + data)
        except (socket.error, BrokenPipeError) as e:
            self.get_logger().warn(f'Send failed (cam {cam_id}): {e}. Reconnecting...')
            try:
                self.sock.close()
            except Exception:
                pass
            self._connect()
            # retry หนึ่งครั้ง
            try:
                self.sock.sendall(header + data)
            except Exception as e2:
                self.get_logger().error(f'Resend after reconnect failed: {e2}')

    def _tick(self):
        for cam_i, cap in zip(self.cam_indices, self.caps):
            if not cap or not cap.isOpened():
                continue
            ret, frame = cap.read()
            if not ret or frame is None:
                self.get_logger().warn(f'Failed to read frame from cam {cam_i}')
                continue
            self._send_frame(cam_i, frame)

    def destroy_node(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        for cap in self.caps:
            try:
                if cap:
                    cap.release()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DualCamSendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
