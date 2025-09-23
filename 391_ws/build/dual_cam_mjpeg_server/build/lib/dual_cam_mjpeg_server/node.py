#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual USB camera publisher with MJPEG HTTP server.
- เปิดกล้อง 2 ตัว
- สตรีมผ่านเว็บ: http://<IP>:8080/ (cam0.mjpg, cam1.mjpg)
- publish topic: /cam0/image/compressed, /cam1/image/compressed
"""

import threading
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Dual Cam + ROS Topic</title>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <style>
    body {
      margin:0; padding:0; height:100vh; background:#0b0b0f; color:#eee;
      display:grid; grid-template-rows:60% 40%; font-family:system-ui;
    }
    .cams {
      display:grid; grid-template-columns:1fr 1fr; gap:8px; padding:8px;
    }
    .card {
      background:#14141a; border-radius:12px; padding:8px;
      display:flex; flex-direction:column; align-items:center; justify-content:center;
    }
    img {
      max-width:100%; max-height:100%; border-radius:8px; background:#000;
    }
    .topics {
      background:#111; padding:12px; overflow:auto;
    }
    pre {
      background:#222; padding:8px; border-radius:8px; font-size:14px;
    }
  </style>
  <script src="https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js"></script>
</head>
<body>
  <div class="cams">
    <div class="card"><h3>cam0</h3><img src="/cam0.mjpg"></div>
    <div class="card"><h3>cam1</h3><img src="/cam1.mjpg"></div>
  </div>
  <div class="topics">
    <h3>ROS Topic Viewer</h3>
    <pre id="topicData">Connecting...</pre>
  </div>

  <script>
    // Connect to rosbridge websocket
    var ros = new ROSLIB.Ros({url : "ws://"+window.location.hostname+":9090"});
    ros.on('connection', function() {
      document.getElementById("topicData").textContent = "Connected to rosbridge...";
    });
    ros.on('error', function(e) {
      document.getElementById("topicData").textContent = "Error connecting to rosbridge!";
    });
    ros.on('close', function() {
      document.getElementById("topicData").textContent = "Connection closed.";
    });

    // Subscribe example: std_msgs/String on /chatter (เปลี่ยนได้)
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : "/chatter",
      messageType : "std_msgs/String"
    });

    listener.subscribe(function(message) {
      document.getElementById("topicData").textContent =
        JSON.stringify(message, null, 2);
    });
  </script>
</body>
</html>
"""



def _mjpeg_stream(get_frame, fps):
    boundary = b"--frame"
    delay = 1.0 / max(1, fps)
    while True:
        buf = get_frame()
        if buf is not None:
            yield boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n"
        time.sleep(delay)


class DualCamMJPEG(Node):
    def __init__(self):
        super().__init__('dual_cam_mjpeg_node')

        # ---- Parameters ----
        self.declare_parameter('cam0_device', 0)
        self.declare_parameter('cam1_device', 1)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8080)

        self.dev0 = self._parse_dev(self.get_parameter('cam0_device').value)
        self.dev1 = self._parse_dev(self.get_parameter('cam1_device').value)
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.jpeg_q = int(self.get_parameter('jpeg_quality').value)
        self.http_host = str(self.get_parameter('http_host').value)
        self.http_port = int(self.get_parameter('http_port').value)

        self.get_logger().info(
            f"Starting cams cam0={self.dev0}, cam1={self.dev1}, "
            f"{self.w}x{self.h}@{self.fps}, jpeg_q={self.jpeg_q}, http={self.http_host}:{self.http_port}"
        )

        # ---- Open cameras ----
        self.cap0 = cv2.VideoCapture(self.dev0, cv2.CAP_V4L2)
        self.cap1 = cv2.VideoCapture(self.dev1, cv2.CAP_V4L2)
        for cap in (self.cap0, self.cap1):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
            cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap0.isOpened():
            self.get_logger().error('Failed to open cam0')
        if not self.cap1.isOpened():
            self.get_logger().error('Failed to open cam1')

        # ---- Buffers ----
        self._lock0 = threading.Lock()
        self._lock1 = threading.Lock()
        self._jpeg0 = None
        self._jpeg1 = None

        # ---- Publishers ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub0 = self.create_publisher(CompressedImage, '/cam0/image/compressed', qos)
        self.pub1 = self.create_publisher(CompressedImage, '/cam1/image/compressed', qos)

        # ---- Timer ----
        period = 1.0 / max(1, self.fps)
        self.timer = self.create_timer(period, self._tick)

        # ---- Flask server ----
        self.app = Flask(__name__)

        @self.app.route('/')
        def index():
            return render_template_string(HTML)

        @self.app.route('/cam0.mjpg')
        def cam0_stream():
            return Response(_mjpeg_stream(self._get_jpeg0, self.fps),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/cam1.mjpg')
        def cam1_stream():
            return Response(_mjpeg_stream(self._get_jpeg1, self.fps),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        self._flask_thread = threading.Thread(
            target=self.app.run,
            kwargs={'host': self.http_host, 'port': self.http_port,
                    'debug': False, 'use_reloader': False},
            daemon=True
        )
        self._flask_thread.start()
        self.get_logger().info("HTTP server started.")

    @staticmethod
    def _parse_dev(v):
        try:
            return int(v)
        except Exception:
            return v

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q]

        ok0, f0 = self.cap0.read()
        if ok0:
            ok, buf = cv2.imencode('.jpg', f0, encode_param)
            if ok:
                jb = buf.tobytes()
                with self._lock0:
                    self._jpeg0 = jb
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.format = 'jpeg'
                msg.data = jb
                self.pub0.publish(msg)

        ok1, f1 = self.cap1.read()
        if ok1:
            ok, buf = cv2.imencode('.jpg', f1, encode_param)
            if ok:
                jb = buf.tobytes()
                with self._lock1:
                    self._jpeg1 = jb
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.format = 'jpeg'
                msg.data = jb
                self.pub1.publish(msg)

    def _get_jpeg0(self):
        with self._lock0:
            return self._jpeg0

    def _get_jpeg1(self):
        with self._lock1:
            return self._jpeg1

    def destroy_node(self):
        for cap in (self.cap0, self.cap1):
            try:
                cap.release()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DualCamMJPEG()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
