#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RBE201TC 智能小车 Web 交互式控制中心 (Web Controller Dashboard)
专为地平线 RDK X5 (Linux Ubuntu 22.04 LTS, aarch64) 定制开发。

功能特性:
1. 电机运动控制: 支持 WASD 键盘遥控、触控 D-Pad、长按行驶/松手即停、点动微调与空格急停。
2. 传感器遥测: 四路 TCRT5000 循迹探头 (X1~X4) 实时状态刷新。
3. 连续视频流 (MJPEG): 320x240 @ 15~20fps 低时延视频推流 (/video_feed)，优化无线热点传输。
4. 双模式图传切换: 支持【彩色原图】与【OpenCV 赛道二值化黑白流】无缝切换，板端实时处理。
5. 网页抓拍直存电脑: 前端 Canvas/Blob 自动命名 (snap_YYYYMMDD_HHMMSS.jpg) 保存至操作者电脑。
6. 硬件安全闭环: 严密管理 libsrcampy 硬件生命周期，支持 SIGINT/SIGTERM/atexit 安全释放 CSI 与 I2C。

访问地址: http://<小车IP>:8080 (如 http://172.20.10.2:8080)
"""

import os
import sys
import time
import json
import threading
import signal
import atexit
from datetime import datetime
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import urllib.parse

# 硬件模块导入检测
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import Hobot.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

try:
    from hobot_vio import libsrcampy as srcampy
    import cv2
    import numpy as np
    CAM_AVAILABLE = True
except ImportError:
    CAM_AVAILABLE = False


# ==========================================
# 摄像头视频流采集管理器 (Camera Streamer)
# ==========================================
class CameraStreamer:
    """
    IMX219 (CSI-CAM0) 硬件视频流常驻引擎:
    - 负责 libsrcampy 的统一生命周期管理 (单例开启，退出安全释放)
    - 320x240 分辨率降采样与 15~20fps 节流
    - 支持 'color' (彩色原图) 与 'binary' (OpenCV 赛道 Otsu 二值化)
    """
    def __init__(self, target_fps=18, width=320, height=240):
        self.target_fps = target_fps
        self.width = width
        self.height = height
        self.frame_interval = 1.0 / target_fps
        
        self.cam = None
        self.cam_opened = False
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
        self.mode = "color"  # "color" | "binary"
        self.latest_jpeg = None
        self.latest_frame_time = 0
        self.fps = 0.0
        self.frame_count = 0
        
        # 抓拍图片持久化路径 (板端备份)
        self.last_snap_path = "/home/sunrise/RBE201TC/web_latest_snap.jpg"
        self.last_snap_time = None

        if CAM_AVAILABLE:
            self.start()
        else:
            print("[WARN] libsrcampy / OpenCV 不可用，启动虚拟测试视频流")
            self.start_sim()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._capture_worker, name="CameraWorker", daemon=True)
        self.thread.start()

    def start_sim(self):
        """虚拟流：用于无硬件开发或摄像头异常时的链路模拟"""
        self.running = True
        self.thread = threading.Thread(target=self._sim_worker, name="CameraSimWorker", daemon=True)
        self.thread.start()

    def set_mode(self, mode):
        if mode in ["color", "binary"]:
            with self.lock:
                self.mode = mode
            print(f"[CAM] 视觉图传模式切换为: {mode.upper()}")
            return True
        return False

    def get_mode(self):
        with self.lock:
            return self.mode

    def get_latest_jpeg(self):
        with self.lock:
            return self.latest_jpeg

    def _capture_worker(self):
        print("[CAM] 正在启动 IMX219 硬件加速管道 (CAM0, i2c-6 @ 0x10)...")
        try:
            self.cam = srcampy.Camera()
            # open_cam(pipe_id=0, video_idx=-1, fps=30, width_list=[1920], height_list=[1088], sensor_h=1080, sensor_w=1920)
            ret = self.cam.open_cam(0, -1, 30, [1920], [1088], 1080, 1920)
            if ret != 0:
                print(f"[ERR] 摄像头 open_cam 失败，返回码: {ret}，降级至模拟流")
                self.start_sim()
                return

            self.cam_opened = True
            print("[OK] IMX219 摄像头通道开启成功，等待曝光传感器稳定...")
            time.sleep(0.8) # 启动阶段缓冲曝光
            print("[OK] 视觉视频流管道已进入就绪状态 (320x240 @ 18fps)")

            fps_calc_time = time.time()
            fps_count = 0

            while self.running:
                loop_start = time.time()
                img_bytes = self.cam.get_img(2, 1920, 1088)
                if img_bytes is None:
                    time.sleep(0.01)
                    continue

                raw_arr = np.frombuffer(img_bytes, dtype=np.uint8).reshape((1088 * 3 // 2, 1920))

                with self.lock:
                    active_mode = self.mode

                # 双模式图像处理流水线
                if active_mode == "binary":
                    # 赛道二值化极速通道:
                    # NV12 前 1080 行即为亮度 Y 通道 (纯灰度图)，省去 cv2.cvtColor(BGR) 的 CPU 消耗
                    y_gray = raw_arr[:1080, :1920]
                    small_gray = cv2.resize(y_gray, (self.width, self.height), interpolation=cv2.INTER_AREA)
                    # 高斯滤波抑制反光杂波
                    blurred = cv2.GaussianBlur(small_gray, (5, 5), 0)
                    # Otsu 赛道大津二值化
                    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    processed_img = binary
                else:
                    # 彩色原图通道: NV12 转 BGR 后下采样
                    bgr = cv2.cvtColor(raw_arr, cv2.COLOR_YUV2BGR_NV12)[:1080, :1920]
                    small_bgr = cv2.resize(bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
                    processed_img = small_bgr

                # JPEG 压缩编码 (品质 75，单帧仅约 8~12KB，低延迟抗丢包)
                ret_enc, jpeg_data = cv2.imencode(".jpg", processed_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ret_enc:
                    jpeg_bytes = jpeg_data.tobytes()
                    with self.lock:
                        self.latest_jpeg = jpeg_bytes
                        self.latest_frame_time = time.time()
                        self.frame_count += 1
                        fps_count += 1

                # 计算实际图传帧率
                now = time.time()
                if now - fps_calc_time >= 1.0:
                    self.fps = round(fps_count / (now - fps_calc_time), 1)
                    fps_count = 0
                    fps_calc_time = now

                # 严格节流以维持目标 FPS (15~20fps)
                elapsed = time.time() - loop_start
                sleep_dur = self.frame_interval - elapsed
                if sleep_dur > 0:
                    time.sleep(sleep_dur)

        except Exception as e:
            print(f"[ERR] 摄像头采集线程异常: {e}")
        finally:
            self.close()

    def _sim_worker(self):
        """模拟测试流，生成包含时间戳与网格的动态画面"""
        fps_calc_time = time.time()
        fps_count = 0
        angle = 0
        while self.running:
            loop_start = time.time()
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            frame[:] = (25, 35, 45) # 背景色

            # 绘制赛道模拟引导线
            cv2.line(frame, (40, 220), (140, 40), (200, 200, 200), 10)
            cv2.line(frame, (280, 220), (180, 40), (200, 200, 200), 10)
            
            # 动态模拟中心线
            cx = int(160 + 40 * np.sin(angle))
            angle += 0.08
            cv2.circle(frame, (cx, 140), 12, (56, 189, 248), -1)

            with self.lock:
                active_mode = self.mode

            if active_mode == "binary":
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, processed_img = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
            else:
                cv2.putText(frame, "SIMULATION STREAM", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(frame, datetime.now().strftime("%H:%M:%S.%f")[:12], (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (148, 163, 184), 1)
                processed_img = frame

            ret_enc, jpeg_data = cv2.imencode(".jpg", processed_img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ret_enc:
                with self.lock:
                    self.latest_jpeg = jpeg_data.tobytes()
                    self.latest_frame_time = time.time()
                    self.frame_count += 1
                    fps_count += 1

            now = time.time()
            if now - fps_calc_time >= 1.0:
                self.fps = round(fps_count / (now - fps_calc_time), 1)
                fps_count = 0
                fps_calc_time = now

            elapsed = time.time() - loop_start
            sleep_dur = self.frame_interval - elapsed
            if sleep_dur > 0:
                time.sleep(sleep_dur)

    def snap_save_local(self):
        """将当前最新帧保存到板端本地，并返回时间与字节数据"""
        with self.lock:
            jpeg_bytes = self.latest_jpeg
            mode_name = self.mode

        if jpeg_bytes is None:
            return False, "当前视频流无就绪帧", None

        try:
            with open(self.last_snap_path, "wb") as f:
                f.write(jpeg_bytes)
            self.last_snap_time = datetime.now().strftime("%H:%M:%S")
            return True, self.last_snap_path, jpeg_bytes
        except Exception as e:
            return False, str(e), None

    def close(self):
        """严格维护释放生命周期，防止 CSI 硬件死锁"""
        self.running = False
        with self.lock:
            if self.cam_opened and self.cam is not None:
                try:
                    self.cam.close_cam()
                    print("[OK] IMX219 摄像头已安全关闭并释放 CSI/I2C 总线")
                except Exception as e:
                    print(f"[WARN] 摄像头释放异常: {e}")
                finally:
                    self.cam_opened = False
                    self.cam = None


# ==========================================
# 硬件驱动管理器 (带安全看门狗与线程锁)
# ==========================================
class HardwareManager:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.serial_lock = threading.Lock()
        self.connected = False
        
        self.default_pwm = 1800
        self.current_action = "stop"
        self.tracking_pins = {"X1": 15, "X2": 18, "X3": 16, "X4": 11}
        
        # 初始化摄像头流引擎
        self.cam_streamer = CameraStreamer(target_fps=18, width=320, height=240)
        
        # 安全看门狗：如果非 stop 动作且超过 1.8 秒未续期，自动安全刹车
        self.last_heartbeat = time.time()
        self.watchdog_enabled = True
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)

        self._init_serial()
        self._init_gpio()
        self.watchdog_thread.start()

    def _init_serial(self):
        if not SERIAL_AVAILABLE:
            print("[WARN] pyserial 未安装")
            return
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            print(f"[OK] 串口驱动连接成功: {self.port} @ {self.baudrate}")
            # 初始化电机驱动板为 TT 电机无编码器模式 (mtype:4, PWM 满量程 0~3600)
            with self.serial_lock:
                self.ser.write(b"$mtype:4#\r\n")
                time.sleep(0.05)
                self.ser.write(b"$upload:0,0,0#\r\n")
                time.sleep(0.05)
                self.ser.write(b"$pwm:0,0,0,0#\r\n")
                self.ser.flush()
            print("[OK] 电机驱动板已初始化 (mtype:4 TT普通电机无编码器模式)")
        except Exception as e:
            print(f"[WARN] 串口打开失败 ({self.port}): {e}")
            self.connected = False

    def _init_gpio(self):
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BOARD)
                for pin in self.tracking_pins.values():
                    GPIO.setup(pin, GPIO.IN)
                print("[OK] 四路红外循迹传感器 GPIO 初始化成功 (BOARD 模式)")
            except Exception as e:
                print(f"[WARN] GPIO 初始化失败: {e}")

    def _watchdog_loop(self):
        while True:
            time.sleep(0.2)
            if self.watchdog_enabled and self.current_action != "stop":
                if time.time() - self.last_heartbeat > 1.8:
                    print(f"[WATCHDOG] 超时未收到控制心跳，自动触发安全刹车！")
                    self.stop()

    def send_pwm(self, m1, m2, m3, m4):
        with self.serial_lock:
            if not self.ser or not self.ser.is_open:
                return False
            # 钳位在 -3600 ~ +3600
            m1 = int(max(-3600, min(3600, m1)))
            m2 = int(max(-3600, min(3600, m2)))
            m3 = int(max(-3600, min(3600, m3)))
            m4 = int(max(-3600, min(3600, m4)))
            cmd = f"$pwm:{m1},{m2},{m3},{m4}#"
            try:
                self.ser.write(cmd.encode("utf-8"))
                self.ser.flush()
                return True
            except Exception as e:
                print(f"[ERR] 串口写入错误: {e}")
                return False

    def drive(self, action, pwm=None):
        speed = int(pwm) if pwm is not None else self.default_pwm
        speed = max(500, min(3600, speed))
        self.last_heartbeat = time.time()
        self.current_action = action

        if action == "forward":
            self.send_pwm(speed, speed, speed, speed)
        elif action == "backward":
            self.send_pwm(-speed, -speed, -speed, -speed)
        elif action == "left":
            self.send_pwm(-speed, -speed, speed, speed)
        elif action == "right":
            self.send_pwm(speed, speed, -speed, -speed)
        elif action == "stop":
            self.stop()
        else:
            return False
        return True

    def stop(self):
        self.current_action = "stop"
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    for _ in range(2):
                        self.ser.write(b"$pwm:0,0,0,0#\r\n")
                        self.ser.write(b"$spd:0,0,0,0#\r\n")
                        self.ser.flush()
                        time.sleep(0.01)
                except Exception:
                    pass

    def read_tracking(self):
        result = {}
        if GPIO_AVAILABLE:
            for name, pin in self.tracking_pins.items():
                try:
                    result[name] = GPIO.input(pin)
                except Exception:
                    result[name] = -1
        else:
            result = {"X1": 1, "X2": 0, "X3": 0, "X4": 1} # 模拟测试值
        return result

    def cleanup(self):
        print("\n[CLEANUP] 开始释放底层软硬件资源...")
        self.stop()
        if self.cam_streamer:
            self.cam_streamer.close()
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        print("[CLEANUP] 底层资源释放完毕")


# 全局硬件实例
hw = HardwareManager()


# ==========================================
# 前端 HTML/CSS/JS 页面模板 (深色科技风)
# ==========================================
INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>RBE201TC 智能车机载控制中心</title>
  <style>
    :root {
      --bg-primary: #0f172a;
      --bg-card: #1e293b;
      --accent: #38bdf8;
      --accent-hover: #0ea5e9;
      --danger: #ef4444;
      --danger-hover: #dc2626;
      --success: #22c55e;
      --warning: #f59e0b;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --border: #334155;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
      background-color: var(--bg-primary);
      color: var(--text-main);
      padding: 16px;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    .container {
      width: 100%;
      max-width: 1040px;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    /* 顶部导航与状态栏 */
    header {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px 20px;
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
    }

    h1 {
      font-size: 1.2rem;
      font-weight: 700;
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .status-badges {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }

    .badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 10px;
      border-radius: 9999px;
      font-size: 0.8rem;
      background: #0f172a;
      border: 1px solid var(--border);
    }
    .badge-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--danger);
    }
    .badge-dot.active { background: var(--success); }
    .badge-dot.pulse { animation: blink 1.2s infinite ease-in-out; }
    @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.3; } }

    /* 主内容布局网格 */
    .grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 16px;
    }
    @media (min-width: 840px) {
      .grid { grid-template-columns: 1.05fr 0.95fr; }
    }

    .card {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 18px;
      display: flex;
      flex-direction: column;
      gap: 14px;
    }

    .card-title {
      font-size: 0.95rem;
      font-weight: 600;
      color: var(--accent);
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid var(--border);
      padding-bottom: 8px;
    }

    /* 虚拟方向控制盘 (D-Pad) */
    .dpad-container {
      display: grid;
      grid-template-columns: repeat(3, 75px);
      grid-template-rows: repeat(3, 75px);
      gap: 12px;
      justify-content: center;
      margin: 8px 0;
      user-select: none;
    }

    .dpad-btn {
      background: #334155;
      color: #fff;
      border: 2px solid #475569;
      border-radius: 14px;
      font-size: 1.4rem;
      font-weight: bold;
      display: flex;
      align-items: center;
      justify-content: center;
      cursor: pointer;
      box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.4);
      transition: all 0.1s ease;
      touch-action: none;
    }
    .dpad-btn:active, .dpad-btn.active {
      background: var(--accent);
      border-color: #7dd3fc;
      transform: scale(0.95);
      box-shadow: 0 0 12px var(--accent);
    }
    .btn-up { grid-column: 2; grid-row: 1; }
    .btn-left { grid-column: 1; grid-row: 2; }
    .btn-stop-center { grid-column: 2; grid-row: 2; background: #450a0a; border-color: var(--danger); font-size: 0.9rem; }
    .btn-stop-center:active { background: var(--danger); }
    .btn-right { grid-column: 3; grid-row: 2; }
    .btn-down { grid-column: 2; grid-row: 3; }

    /* 急停按钮 */
    .emergency-btn {
      background: var(--danger);
      color: white;
      font-weight: 700;
      font-size: 1.05rem;
      padding: 12px;
      border-radius: 10px;
      border: none;
      cursor: pointer;
      box-shadow: 0 4px 14px rgba(239, 68, 68, 0.4);
      transition: background 0.15s, transform 0.1s;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
    }
    .emergency-btn:active {
      background: var(--danger-hover);
      transform: scale(0.98);
    }

    /* 模式切换与滑块 */
    .controls-row {
      display: flex;
      flex-direction: column;
      gap: 10px;
      background: #0f172a;
      padding: 12px;
      border-radius: 8px;
      border: 1px solid var(--border);
    }

    .slider-group {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .slider-group input[type="range"] {
      flex: 1;
      height: 6px;
      accent-color: var(--accent);
      cursor: pointer;
    }
    .slider-val {
      font-family: monospace;
      font-size: 0.95rem;
      font-weight: bold;
      color: var(--accent);
      width: 105px;
      text-align: right;
    }

    .mode-tags {
      display: flex;
      gap: 8px;
      font-size: 0.85rem;
    }
    .mode-tag {
      padding: 4px 10px;
      border-radius: 6px;
      border: 1px solid var(--border);
      background: #1e293b;
      color: var(--text-main);
      cursor: pointer;
      font-size: 0.82rem;
      transition: all 0.15s;
    }
    .mode-tag:hover { border-color: var(--accent); }
    .mode-tag.active {
      border-color: var(--accent);
      background: rgba(56, 189, 248, 0.18);
      color: var(--accent);
      font-weight: 600;
    }

    /* 传感器面板 */
    .sensors-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 10px;
      text-align: center;
    }
    .sensor-item {
      background: #0f172a;
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 10px 4px;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 6px;
    }
    .sensor-light {
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: #1e293b;
      border: 2px solid #475569;
      transition: all 0.2s;
    }
    .sensor-light.white-ground {
      background: #22c55e;
      border-color: #86efac;
      box-shadow: 0 0 10px #22c55e;
    }
    .sensor-light.black-line {
      background: #0f172a;
      border-color: #64748b;
    }
    .sensor-name {
      font-size: 0.82rem;
      font-weight: 600;
    }
    .sensor-state-txt {
      font-size: 0.75rem;
      color: var(--text-muted);
    }

    /* 视觉监控面板 */
    .camera-box {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 12px;
    }
    .video-viewport {
      width: 100%;
      max-width: 400px;
      aspect-ratio: 4 / 3;
      background: #000;
      border: 2px solid var(--border);
      border-radius: 10px;
      overflow: hidden;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
    }
    .video-viewport img {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }
    .video-osd {
      position: absolute;
      top: 8px;
      left: 8px;
      display: flex;
      gap: 6px;
    }
    .osd-pill {
      background: rgba(15, 23, 42, 0.75);
      border: 1px solid rgba(255,255,255,0.15);
      padding: 2px 8px;
      border-radius: 4px;
      font-size: 0.7rem;
      font-family: monospace;
      color: #38bdf8;
    }

    .cam-btn-group {
      display: flex;
      width: 100%;
      gap: 10px;
      flex-wrap: wrap;
    }

    .btn-cam-action {
      flex: 1;
      min-width: 140px;
      background: #0284c7;
      color: white;
      border: none;
      border-radius: 8px;
      padding: 10px 14px;
      font-size: 0.88rem;
      font-weight: 600;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: background 0.15s, transform 0.08s;
    }
    .btn-cam-action:hover { background: #0369a1; }
    .btn-cam-action:active { transform: scale(0.97); }
    .btn-cam-action.secondary {
      background: #334155;
      color: #f1f5f9;
      border: 1px solid #475569;
    }
    .btn-cam-action.secondary:hover { background: #475569; }

    /* 控制日志小窗口 */
    .log-box {
      background: #090d16;
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 10px;
      font-family: monospace;
      font-size: 0.76rem;
      height: 105px;
      overflow-y: auto;
      color: #a5f3fc;
    }
    .log-entry { margin-bottom: 3px; }
    .log-time { color: var(--text-muted); margin-right: 6px; }

    .help-hint {
      font-size: 0.78rem;
      color: var(--text-muted);
      line-height: 1.45;
    }
    kbd {
      background: #334155;
      padding: 2px 5px;
      border-radius: 4px;
      color: #fff;
      font-family: monospace;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>🚗 RBE201TC 智能车机载控制中心</h1>
      <div class="status-badges">
        <div class="badge">
          <span id="dot-serial" class="badge-dot"></span>
          <span>串口: <span id="txt-serial">检测中</span></span>
        </div>
        <div class="badge">
          <span id="dot-stream" class="badge-dot active pulse"></span>
          <span>图传: <span id="txt-fps">-- fps</span></span>
        </div>
        <div class="badge">
          <span id="dot-action" class="badge-dot"></span>
          <span>状态: <span id="txt-action">STOP</span></span>
        </div>
      </div>
    </header>

    <div class="grid">
      <!-- 左列: 驾驶控制面板 -->
      <div class="card">
        <div class="card-title">
          <span>🎮 运动驱动控制</span>
          <span style="font-size:0.8rem; color:var(--text-muted);">键盘 WASD / 触控长按</span>
        </div>

        <!-- 调速与模式 -->
        <div class="controls-row">
          <div class="slider-group">
            <span style="font-size:0.85rem; color:var(--text-muted);">PWM速度:</span>
            <input type="range" id="pwm-slider" min="1000" max="3600" step="50" value="1800">
            <span id="pwm-val" class="slider-val">1800 (50%)</span>
          </div>
          <div style="display:flex; gap:6px; flex-wrap:wrap;">
            <button class="mode-tag" type="button" onclick="setPreset(1600)">🐢 慢速 1600</button>
            <button class="mode-tag" type="button" onclick="setPreset(2200)">🚗 巡航 2200</button>
            <button class="mode-tag" type="button" onclick="setPreset(2800)">⚡ 高速 2800</button>
          </div>
          <div class="mode-tags" style="margin-top:2px;">
            <button class="mode-tag active" id="mode-hold" type="button">按住长按运行 (松手即停)</button>
            <button class="mode-tag" id="mode-pulse" type="button">点击点动微调 (1.2秒)</button>
          </div>
        </div>

        <!-- 十字虚拟控制盘 -->
        <div class="dpad-container">
          <button class="dpad-btn btn-up" data-act="forward">▲</button>
          <button class="dpad-btn btn-left" data-act="left">◀</button>
          <button class="dpad-btn btn-stop-center" data-act="stop">刹</button>
          <button class="dpad-btn btn-right" data-act="right">▶</button>
          <button class="dpad-btn btn-down" data-act="backward">▼</button>
        </div>

        <!-- 急停红色大按钮 -->
        <button class="emergency-btn" id="btn-stop">
          🛑 紧急刹车 (SPACE)
        </button>

        <div class="help-hint">
          💡 <b>键盘操控</b>: <kbd>W</kbd> 前进、<kbd>S</kbd> 后退、<kbd>A</kbd> 左转、<kbd>D</kbd> 右转；松开自动刹停；任意时刻按 <kbd>空格</kbd> 触发紧急制动。
        </div>
      </div>

      <!-- 右列: 传感器 & 视觉监控 -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <!-- 红外循迹状态 -->
        <div class="card">
          <div class="card-title">
            <span>📡 四路循迹传感器遥测 (X1 ~ X4)</span>
            <span style="font-size:0.75rem; color:var(--text-muted);">绿:白底(1) / 暗:黑线(0)</span>
          </div>
          <div class="sensors-grid">
            <div class="sensor-item">
              <span class="sensor-name">X1 (最左)</span>
              <div id="led-x1" class="sensor-light"></div>
              <span id="val-x1" class="sensor-state-txt">--</span>
            </div>
            <div class="sensor-item">
              <span class="sensor-name">X2 (偏左)</span>
              <div id="led-x2" class="sensor-light"></div>
              <span id="val-x2" class="sensor-state-txt">--</span>
            </div>
            <div class="sensor-item">
              <span class="sensor-name">X3 (偏右)</span>
              <div id="led-x3" class="sensor-light"></div>
              <span id="val-x3" class="sensor-state-txt">--</span>
            </div>
            <div class="sensor-item">
              <span class="sensor-name">X4 (最右)</span>
              <div id="led-x4" class="sensor-light"></div>
              <span id="val-x4" class="sensor-state-txt">--</span>
            </div>
          </div>
        </div>

        <!-- 视觉监控面板 (MJPEG 视频流 + 双模式切换 + 本地下载) -->
        <div class="card">
          <div class="card-title">
            <span>📹 IMX219 实时图传与赛道视觉</span>
            <span id="stream-mode-label" style="font-size:0.75rem; color:var(--accent);">彩色原图</span>
          </div>

          <div class="camera-box">
            <!-- 模式切换开关组 -->
            <div style="display:flex; width:100%; justify-content:space-between; align-items:center;">
              <span style="font-size:0.82rem; color:var(--text-muted);">图传模式:</span>
              <div class="mode-tags">
                <button class="mode-tag active" id="btn-mode-color" type="button" onclick="switchCameraMode('color')">🌈 彩色原图</button>
                <button class="mode-tag" id="btn-mode-binary" type="button" onclick="switchCameraMode('binary')">🏁 赛道二值化</button>
              </div>
            </div>

            <!-- 视频流播放视口 -->
            <div class="video-viewport" id="video-box">
              <img id="mjpeg-stream" src="/video_feed" alt="实时视频流加载中...">
              <div class="video-osd">
                <span class="osd-pill">320x240</span>
                <span class="osd-pill" id="osd-mode">COLOR</span>
              </div>
            </div>

            <!-- 操作按钮组: 抓拍直存电脑 & 画面重连 -->
            <div class="cam-btn-group">
              <button class="btn-cam-action" id="btn-download-snap" type="button">
                💾 抓拍并下载到电脑
              </button>
              <button class="btn-cam-action secondary" id="btn-reload-stream" type="button" onclick="reloadStream()">
                🔄 重连视频流
              </button>
            </div>
          </div>
        </div>

        <!-- 控制事件日志 -->
        <div class="card" style="padding:12px;">
          <div class="card-title" style="padding-bottom:4px; font-size:0.88rem;">
            <span>📜 控制链路日志</span>
            <span style="font-size:0.75rem; color:var(--text-muted); cursor:pointer;" onclick="document.getElementById('log-box').innerHTML=''">清空</span>
          </div>
          <div class="log-box" id="log-box"></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    // 全局状态
    let currentMode = 'hold'; // 'hold' | 'pulse'
    let currentPwm = 1800;
    let heartbeatTimer = null;
    let isDriving = false;
    let activeKey = null;
    let currentCamMode = 'color';

    // DOM 元素
    const logBox = document.getElementById('log-box');
    const pwmSlider = document.getElementById('pwm-slider');
    const pwmVal = document.getElementById('pwm-val');
    const modeHold = document.getElementById('mode-hold');
    const modePulse = document.getElementById('mode-pulse');
    const btnStop = document.getElementById('btn-stop');
    const mjpegStream = document.getElementById('mjpeg-stream');
    const btnDownloadSnap = document.getElementById('btn-download-snap');
    const osdMode = document.getElementById('osd-mode');
    const streamModeLabel = document.getElementById('stream-mode-label');
    const btnModeColor = document.getElementById('btn-mode-color');
    const btnModeBinary = document.getElementById('btn-mode-binary');

    function log(msg) {
      const now = new Date().toTimeString().split(' ')[0];
      const div = document.createElement('div');
      div.className = 'log-entry';
      div.innerHTML = `<span class="log-time">[${now}]</span> ${msg}`;
      logBox.appendChild(div);
      logBox.scrollTop = logBox.scrollHeight;
    }

    function updatePwmUI() {
      pwmVal.textContent = `${currentPwm} (${Math.round(currentPwm / 36)}%)`;
    }

    // 调速滑块
    pwmSlider.addEventListener('input', (e) => {
      currentPwm = parseInt(e.target.value);
      updatePwmUI();
    });

    window.setPreset = function(val) {
      currentPwm = val;
      pwmSlider.value = val;
      updatePwmUI();
      log(`⚡ 快捷调速至: PWM ${val} (${Math.round(val / 36)}% 功率)`);
    };

    // 模式切换
    modeHold.addEventListener('click', () => {
      currentMode = 'hold';
      modeHold.classList.add('active');
      modePulse.classList.remove('active');
      log("模式已切换: 按住长按运行 (松开即停)");
    });
    modePulse.addEventListener('click', () => {
      currentMode = 'pulse';
      modePulse.classList.add('active');
      modeHold.classList.remove('active');
      log("模式已切换: 点击点动微调 (每次 1.2 秒)");
    });

    // 发送动作请求 API
    async function sendDrive(action, pulseDuration = 0) {
      try {
        const resp = await fetch('/api/drive', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: action, pwm: currentPwm, duration: pulseDuration })
        });
        const data = await resp.json();
        if (data.status === 'ok') {
          const pct = Math.round(currentPwm / 36);
          log(`指令已执行: <b>${action.toUpperCase()}</b> (PWM ${currentPwm} / ${pct}%)`);
        }
      } catch (err) {
        log(`⚠ 指令发送错误: ${err}`);
      }
    }

    // 开始行驶 (带心跳保障)
    function startDrive(action) {
      if (currentMode === 'pulse') {
        sendDrive(action, 1.2);
        return;
      }
      isDriving = true;
      sendDrive(action);
      clearInterval(heartbeatTimer);
      heartbeatTimer = setInterval(() => {
        if (isDriving) {
          sendDrive(action);
        }
      }, 500);
    }

    // 停止行驶
    function stopDrive() {
      if (!isDriving && currentMode === 'hold') return;
      isDriving = false;
      clearInterval(heartbeatTimer);
      sendDrive('stop');
    }

    // 绑定 D-Pad 按钮事件
    document.querySelectorAll('.dpad-btn').forEach(btn => {
      const act = btn.getAttribute('data-act');
      if (act === 'stop') {
        btn.addEventListener('click', () => stopDrive());
        return;
      }

      btn.addEventListener('pointerdown', (e) => {
        btn.setPointerCapture(e.pointerId);
        btn.classList.add('active');
        startDrive(act);
      });

      const handleRelease = (e) => {
        btn.classList.remove('active');
        if (currentMode === 'hold') {
          stopDrive();
        }
      };

      btn.addEventListener('pointerup', handleRelease);
      btn.addEventListener('pointercancel', handleRelease);
    });

    // 急停按钮
    btnStop.addEventListener('click', () => {
      stopDrive();
      log("🛑 用户触发紧急刹车！");
    });

    // 键盘监听
    const keyMap = {
      'KeyW': 'forward', 'ArrowUp': 'forward',
      'KeyS': 'backward', 'ArrowDown': 'backward',
      'KeyA': 'left', 'ArrowLeft': 'left',
      'KeyD': 'right', 'ArrowRight': 'right'
    };

    window.addEventListener('keydown', (e) => {
      if (e.code === 'Space') {
        e.preventDefault();
        stopDrive();
        log("🛑 空格键紧急制动！");
        return;
      }
      if (keyMap[e.code] && activeKey !== e.code) {
        e.preventDefault();
        activeKey = e.code;
        startDrive(keyMap[e.code]);
      }
    });

    window.addEventListener('keyup', (e) => {
      if (keyMap[e.code] && activeKey === e.code) {
        activeKey = null;
        if (currentMode === 'hold') {
          stopDrive();
        }
      }
    });

    // ==========================================
    // 视觉图传控制: 双模式切换与本地保存
    // ==========================================
    window.switchCameraMode = async function(mode) {
      if (currentCamMode === mode) return;
      currentCamMode = mode;
      
      if (mode === 'binary') {
        btnModeBinary.classList.add('active');
        btnModeColor.classList.remove('active');
        osdMode.textContent = 'BINARY (OTSU)';
        streamModeLabel.textContent = '赛道二值化';
      } else {
        btnModeColor.classList.add('active');
        btnModeBinary.classList.remove('active');
        osdMode.textContent = 'COLOR';
        streamModeLabel.textContent = '彩色原图';
      }

      try {
        await fetch('/api/camera_mode', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ mode: mode })
        });
        log(`📷 图传已切换为: <b>${mode === 'binary' ? '🏁 赛道二值化' : '🌈 彩色原图'}</b>`);
      } catch (err) {
        log(`⚠ 切换图传模式网络错误: ${err}`);
      }
    };

    window.reloadStream = function() {
      mjpegStream.src = '/video_feed?t=' + new Date().getTime();
      log("🔄 已重新请求建立视频推流管道");
    };

    // 抓拍并直接保存到操作者电脑 (HTML5 Canvas -> Blob -> 浏览器本地下载)
    btnDownloadSnap.addEventListener('click', async () => {
      btnDownloadSnap.disabled = true;
      btnDownloadSnap.textContent = "⏳ 正在生成本地文件...";

      try {
        const canvas = document.createElement('canvas');
        const width = mjpegStream.naturalWidth || 320;
        const height = mjpegStream.naturalHeight || 240;
        canvas.width = width;
        canvas.height = height;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(mjpegStream, 0, 0, width, height);

        canvas.toBlob((blob) => {
          if (!blob) {
            log("❌ 生成抓拍 Blob 数据失败");
            btnDownloadSnap.disabled = false;
            btnDownloadSnap.textContent = "💾 抓拍并下载到电脑";
            return;
          }

          // 时间戳格式: snap_YYYYMMDD_HHMMSS.jpg
          const now = new Date();
          const pad = (n) => String(n).padStart(2, '0');
          const timeStr = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
          const filename = `snap_${timeStr}.jpg`;

          // 创建临时 a 标签触发本地下载
          const blobUrl = URL.createObjectURL(blob);
          const link = document.createElement('a');
          link.href = blobUrl;
          link.download = filename;
          document.body.appendChild(link);
          link.click();
          document.body.removeChild(link);
          URL.revokeObjectURL(blobUrl);

          log(`✓ 📸 抓拍成功！文件已存入电脑下载目录: <b>${filename}</b> (${currentCamMode.toUpperCase()})`);

          // 同时异步通知板端也保留一份最新抓拍记录
          fetch('/api/snap', { method: 'POST' }).catch(() => {});

          btnDownloadSnap.disabled = false;
          btnDownloadSnap.textContent = "💾 抓拍并下载到电脑";
        }, 'image/jpeg', 0.95);

      } catch (err) {
        log(`❌ 本地抓拍下载异常: ${err}`);
        btnDownloadSnap.disabled = false;
        btnDownloadSnap.textContent = "💾 抓拍并下载到电脑";
      }
    });

    // 状态轮询 (每 500ms)
    async function pollStatus() {
      try {
        const resp = await fetch('/api/status');
        const data = await resp.json();

        // 串口指示灯
        const dotSerial = document.getElementById('dot-serial');
        const txtSerial = document.getElementById('txt-serial');
        if (data.connected) {
          dotSerial.classList.add('active');
          txtSerial.textContent = "正常";
        } else {
          dotSerial.classList.remove('active');
          txtSerial.textContent = "未就绪";
        }

        // 图传 FPS 与模式
        const txtFps = document.getElementById('txt-fps');
        if (data.fps !== undefined) {
          txtFps.textContent = `${data.fps} fps`;
        }

        // 当前动作
        const txtAction = document.getElementById('txt-action');
        txtAction.textContent = data.current_action.toUpperCase();

        // 循迹探头更新 (黑0白1)
        if (data.tracking) {
          ['X1', 'X2', 'X3', 'X4'].forEach(name => {
            const val = data.tracking[name];
            const led = document.getElementById('led-' + name.toLowerCase());
            const txt = document.getElementById('val-' + name.toLowerCase());
            if (val === 1) {
              led.className = 'sensor-light white-ground';
              txt.textContent = '白底 (1)';
            } else if (val === 0) {
              led.className = 'sensor-light black-line';
              txt.textContent = '黑线 (0)';
            } else {
              led.className = 'sensor-light';
              txt.textContent = '异常';
            }
          });
        }
      } catch (e) {
        // 网络波动静默
      }
    }

    setInterval(pollStatus, 500);
    pollStatus();
    log("🚀 控制台已就绪，低时延 MJPEG 视频流已连通");
  </script>
</body>
</html>
"""


# ==========================================
# HTTP 服务处理器
# ==========================================
class CarHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # 屏蔽高频轮询与推流的控制台刷屏日志
        msg = args[0] if args else ""
        if "/api/status" in msg or "/video_feed" in msg:
            return
        super().log_message(format, *args)

    def do_GET(self):
        url_parts = urllib.parse.urlparse(self.path)
        path = url_parts.path

        if path in ["/", "/index.html"]:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(INDEX_HTML.encode("utf-8"))

        elif path == "/video_feed":
            # 持续输出 MJPEG 视频流 (标准 Multipart 格式)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            streamer = hw.cam_streamer
            try:
                while True:
                    frame = streamer.get_latest_jpeg()
                    if frame is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.05) # ~20fps 节流
            except (BrokenPipeError, ConnectionResetError):
                # 客户端断开连接或刷新网页，优雅退出避免死锁
                pass
            except Exception:
                pass

        elif path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            status_data = {
                "connected": hw.connected,
                "current_action": hw.current_action,
                "default_pwm": hw.default_pwm,
                "tracking": hw.read_tracking(),
                "camera_available": CAM_AVAILABLE,
                "fps": hw.cam_streamer.fps if hw.cam_streamer else 0.0,
                "camera_mode": hw.cam_streamer.get_mode() if hw.cam_streamer else "none",
                "last_snap_time": hw.cam_streamer.last_snap_time if hw.cam_streamer else None
            }
            self.wfile.write(json.dumps(status_data).encode("utf-8"))

        elif path == "/api/image":
            # 返回板端最新备份快照
            snap_path = hw.cam_streamer.last_snap_path if hw.cam_streamer else ""
            if snap_path and os.path.exists(snap_path):
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.end_headers()
                with open(snap_path, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        url_parts = urllib.parse.urlparse(self.path)
        path = url_parts.path
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length)

        if path == "/api/drive":
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
                action = data.get("action", "stop")
                pwm = data.get("pwm", hw.default_pwm)
                duration = data.get("duration", 0)

                if duration and duration > 0:
                    # 脉冲点动
                    def pulse_task():
                        hw.drive(action, pwm)
                        time.sleep(duration)
                        hw.stop()
                    threading.Thread(target=pulse_task, daemon=True).start()
                else:
                    hw.drive(action, pwm)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "action": action}).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))

        elif path == "/api/camera_mode":
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
                mode = data.get("mode", "color")
                if hw.cam_streamer:
                    hw.cam_streamer.set_mode(mode)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "mode": mode}).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))

        elif path == "/api/snap":
            if hw.cam_streamer:
                success, path_or_err, _ = hw.cam_streamer.snap_save_local()
                self.send_response(200 if success else 500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                if success:
                    res = {"status": "ok", "time": hw.cam_streamer.last_snap_time, "path": path_or_err}
                else:
                    res = {"status": "error", "message": path_or_err}
                self.wfile.write(json.dumps(res).encode("utf-8"))
            else:
                self.send_response(500)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


def main():
    port = 8080
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass

    # 注册退出钩子与系统中断信号，确保安全释放硬件
    def handle_exit(signum=None, frame=None):
        print("\n[EXIT] 捕获到退出信号，触发安全关闭流程...")
        hw.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)
    atexit.register(hw.cleanup)

    server_address = ("0.0.0.0", port)
    httpd = ThreadingHTTPServer(server_address, CarHTTPHandler)

    print("=" * 64)
    print("   🚗 RBE201TC 智能小车机载 Web 控制中心 (视觉流升级版) 已启动！")
    print(f"   本地监听端口: {port}")
    print(f"   电脑/手机浏览器访问: http://172.20.10.2:{port}")
    print("   已启用功能: 320x240 @ 18fps MJPEG 视频流 | 彩色/二值化实时切换 | 本地下载")
    print("   按 Ctrl+C 安全停止服务并释放 CSI/I2C/串口硬件")
    print("=" * 64)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        hw.cleanup()
        httpd.server_close()
        print("服务已关闭，系统资源完全释放。")


if __name__ == "__main__":
    main()
