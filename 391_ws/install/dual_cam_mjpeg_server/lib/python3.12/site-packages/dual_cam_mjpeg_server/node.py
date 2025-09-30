#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual USB camera publisher with MJPEG HTTP server + Auto-Reconnect.
- เปิดกล้อง 2 ตัว (รองรับ device เป็น int index หรือ path เช่น /dev/v4l/by-id/...)
- ถอด/เสียบใหม่ จะพยายาม reconnect ให้อัตโนมัติ
- เว็บ: http://<IP>:8080/  (cam0.mjpg, cam1.mjpg)
- ROS2: /cam0/image/compressed, /cam1/image/compressed
"""

import threading
import time
import os
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
    .badge { font-size:12px; opacity:.75; }
    .ok { color:#7fffd4; } .err { color:#ff6b6b; }
  </style>
  <script src="https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js"></script>
</head>
<body>
  <div class="cams">
    <div class="card"><h3>cam0 <span id="s0" class="badge"></span></h3><img src="/cam0.mjpg"></div>
    <div class="card"><h3>cam1 <span id="s1" class="badge"></span></h3><img src="/cam1.mjpg"></div>
  </div>
  <div class="topics">
    <h3>ROS Topic Viewer</h3>
    <pre id="topicData">Connecting...</pre>
  </div>

  <script>
    // rosbridge
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

    var listener = new ROSLIB.Topic({ ros, name : "/chatter", messageType : "std_msgs/String" });
    listener.subscribe(function(message) {
      document.getElementById("topicData").textContent = JSON.stringify(message, null, 2);
    });

    // แสดงสถานะกล้อง (ดึงจาก /status polling เล็กๆ)
    async function refreshStatus(){
      try{
        const r = await fetch('/status'); const js = await r.json();
        for(let i=0;i<2;i++){
          const el = document.getElementById('s'+i);
          if(js['cam'+i]){ el.textContent = 'online'; el.className='badge ok'; }
          else{ el.textContent = 'offline'; el.className='badge err'; }
        }
      }catch(e){}
      setTimeout(refreshStatus, 2000);
    }
    refreshStatus();
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

class CameraWorker:
    """
    Thread worker ดูแลกล้องหนึ่งตัว พร้อม auto-reconnect.
    - ถ้า read fail > N ครั้ง หรือ cap หลุด: release แล้วรอเปิดใหม่
    - รองรับ device เป็น int หรือ string path
    """
    def __init__(self, device, width, height, fps, jpeg_q, log_cb, backend=cv2.CAP_V4L2,
                 reopen_interval=1.0, fail_threshold=10):
        self.device = device
        self.w = width
        self.h = height
        self.fps = fps
        self.jpeg_q = jpeg_q
        self.backend = backend
        self.reopen_interval = reopen_interval
        self.fail_threshold = fail_threshold
        self.log = log_cb

        self.cap = None
        self._jpeg = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._online = False
        self._fails = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @staticmethod
    def _exists_path(dev):
        try:
            return os.path.exists(dev)
        except Exception:
            return False

    def _open_cap(self):
        # ถ้าเป็น string path แต่ไฟล์ยังไม่มี ให้เลี่ยงเปิดทันที
        if isinstance(self.device, str) and not self._exists_path(self.device):
            return False
        cap = cv2.VideoCapture(self.device, self.backend)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap = cap
        self._online = True
        self._fails = 0
        self.log(f"[cam {self.device}] opened {self.w}x{self.h}@{self.fps}")
        return True

    def _close_cap(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None
        self._online = False
        self.log(f"[cam {self.device}] released")

    def _run(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_q)]
        last_open_try = 0.0

        while not self._stop.is_set():
            if self.cap is None or not self.cap.isOpened():
                now = time.time()
                if now - last_open_try >= self.reopen_interval:
                    if self._open_cap():
                        last_open_try = now
                    else:
                        last_open_try = now
                        time.sleep(self.reopen_interval)
                else:
                    time.sleep(0.05)
                continue

            ok, frame = self.cap.read()
            if not ok or frame is None:
                self._fails += 1
                if self._fails >= self.fail_threshold:
                    self.log(f"[cam {self.device}] read fails ({self._fails}) -> reopen")
                    self._close_cap()
                    time.sleep(self.reopen_interval)
                else:
                    time.sleep(0.005)
                continue

            self._fails = 0
            ok2, buf = cv2.imencode('.jpg', frame, encode_param)
            if ok2:
                jb = buf.tobytes()
                with self._lock:
                    self._jpeg = jb
            else:
                # encoding fail นับเป็น fail เบาๆ
                self._fails += 1
                time.sleep(0.002)

        # cleanup
        self._close_cap()

    def get_jpeg(self):
        with self._lock:
            return self._jpeg

    def is_online(self):
        return self._online

    def stop(self):
        self._stop.set()
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass

class DualCamMJPEG(Node):
    def __init__(self):
        super().__init__('dual_cam_mjpeg_node')

        # ---- Parameters ----
        self.declare_parameter('cam0_device', 0)          # แนะนำใช้ /dev/v4l/by-id/xxx
        self.declare_parameter('cam1_device', 2)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('fail_threshold', 10)      # กี่ครั้งถึงจะ reopen
        self.declare_parameter('reopen_interval', 1.0)    # วินาที: ระยะห่างการลองเปิดใหม่

        self.dev0 = self._parse_dev(self.get_parameter('cam0_device').value)
        self.dev1 = self._parse_dev(self.get_parameter('cam1_device').value)
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.jpeg_q = int(self.get_parameter('jpeg_quality').value)
        self.http_host = str(self.get_parameter('http_host').value)
        self.http_port = int(self.get_parameter('http_port').value)
        self.fail_threshold = int(self.get_parameter('fail_threshold').value)
        self.reopen_interval = float(self.get_parameter('reopen_interval').value)

        self.get_logger().info(
            f"Start cams cam0={self.dev0}, cam1={self.dev1}, "
            f"{self.w}x{self.h}@{self.fps}, jpeg_q={self.jpeg_q}, "
            f"http={self.http_host}:{self.http_port}, "
            f"reopen={self.reopen_interval}s, fail_threshold={self.fail_threshold}"
        )

        # ---- Camera workers (auto-reconnect) ----
        self.cam0 = CameraWorker(
            self.dev0, self.w, self.h, self.fps, self.jpeg_q,
            log_cb=lambda s: self.get_logger().info(s),
            reopen_interval=self.reopen_interval, fail_threshold=self.fail_threshold
        )
        self.cam1 = CameraWorker(
            self.dev1, self.w, self.h, self.fps, self.jpeg_q,
            log_cb=lambda s: self.get_logger().info(s),
            reopen_interval=self.reopen_interval, fail_threshold=self.fail_threshold
        )

        # ---- Publishers ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub0 = self.create_publisher(CompressedImage, '/cam0/image/compressed', qos)
        self.pub1 = self.create_publisher(CompressedImage, '/cam1/image/compressed', qos)

        # ---- Timer: publish จาก buffer ล่าสุดของ worker ----
        period = 1.0 / max(1, self.fps)
        self.timer = self.create_timer(period, self._tick)

        # ---- Flask server ----
        self.app = Flask(__name__)

        @self.app.route('/')
        def index():
            return HTML

        @self.app.route('/cam0.mjpg')
        def cam0_stream():
            return Response(_mjpeg_stream(self.cam0.get_jpeg, self.fps),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/cam1.mjpg')
        def cam1_stream():
            return Response(_mjpeg_stream(self.cam1.get_jpeg, self.fps),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/status')
        def status():
            return {'cam0': bool(self.cam0.is_online()),
                    'cam1': bool(self.cam1.is_online())}

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
            return str(v)

    def _tick(self):
        stamp = self.get_clock().now().to_msg()

        # cam0
        jb0 = self.cam0.get_jpeg()
        if jb0 is not None:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.format = 'jpeg'
            msg.data = jb0
            self.pub0.publish(msg)

        # cam1
        jb1 = self.cam1.get_jpeg()
        if jb1 is not None:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.format = 'jpeg'
            msg.data = jb1
            self.pub1.publish(msg)

    def destroy_node(self):
        try:
            self.cam0.stop()
            self.cam1.stop()
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
