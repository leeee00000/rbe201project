#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RBE201TC 智能小车 Web 交互式控制中心 (Web Controller Dashboard)
单文件即开即用，直接运行在 RDK X5 板端，提供浏览器图形化点击控制、
键盘遥控、循迹传感器遥测监控与 IMX219 摄像头抓拍回传。

访问地址: http://<小车IP>:8080 (如 http://172.20.10.2:8080)
"""

import os
import sys
import time
import json
import threading
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
        self.last_snap_path = "/home/sunrise/RBE201TC/web_latest_snap.jpg"
        self.last_snap_time = None
        
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

    def snap_photo(self):
        if not CAM_AVAILABLE:
            return False, "IMX219 / libsrcampy 依赖不可用"
        try:
            cam = srcampy.Camera()
            ret = cam.open_cam(0, -1, 30, [1920], [1088], 1080, 1920)
            if ret != 0:
                return False, f"摄像头打开失败 (错误码 {ret})"
            time.sleep(0.8)
            img = cam.get_img(2, 1920, 1088)
            if img is not None:
                arr = np.frombuffer(img, dtype=np.uint8).reshape((1088 * 3 // 2, 1920))
                bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_NV12)[:1080, :1920]
                cv2.imwrite(self.last_snap_path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
                cam.close_cam()
                self.last_snap_time = time.strftime("%H:%M:%S")
                return True, self.last_snap_path
            else:
                cam.close_cam()
                return False, "未能抓取到图像帧"
        except Exception as e:
            return False, str(e)


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
      max-width: 960px;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    /* 顶部导航与状态栏 */
    header {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 16px 20px;
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
    }

    h1 {
      font-size: 1.25rem;
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

    /* 主内容布局网格 */
    .grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 16px;
    }
    @media (min-width: 768px) {
      .grid { grid-template-columns: 1.1fr 0.9fr; }
    }

    .card {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 20px;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    .card-title {
      font-size: 1rem;
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
      margin: 10px 0;
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
      font-size: 1.1rem;
      padding: 14px;
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
      gap: 12px;
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
      font-size: 1rem;
      font-weight: bold;
      color: var(--accent);
      width: 45px;
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
      cursor: pointer;
    }
    .mode-tag.active {
      border-color: var(--accent);
      background: rgba(56, 189, 248, 0.15);
      color: var(--accent);
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
      font-size: 0.85rem;
      font-weight: 600;
    }
    .sensor-state-txt {
      font-size: 0.75rem;
      color: var(--text-muted);
    }

    /* 视觉抓拍面板 */
    .camera-box {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 12px;
    }
    .preview-frame {
      width: 100%;
      height: 200px;
      background: #090d16;
      border: 1px dashed var(--border);
      border-radius: 8px;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
      position: relative;
    }
    .preview-frame img {
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: none;
    }
    .preview-placeholder {
      color: var(--text-muted);
      font-size: 0.85rem;
    }

    .btn-snap {
      background: #0284c7;
      color: white;
      border: none;
      border-radius: 8px;
      padding: 10px 20px;
      font-size: 0.95rem;
      font-weight: 600;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 8px;
      transition: background 0.15s;
    }
    .btn-snap:hover { background: #0369a1; }

    /* 控制日志小窗口 */
    .log-box {
      background: #090d16;
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 10px;
      font-family: monospace;
      font-size: 0.78rem;
      height: 110px;
      overflow-y: auto;
      color: #a5f3fc;
    }
    .log-entry { margin-bottom: 3px; }
    .log-time { color: var(--text-muted); margin-right: 6px; }

    .help-hint {
      font-size: 0.8rem;
      color: var(--text-muted);
      line-height: 1.5;
    }
    kbd {
      background: #334155;
      padding: 2px 6px;
      border-radius: 4px;
      color: #fff;
      font-family: monospace;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>🚗 RBE201TC 智能车控制中心</h1>
      <div class="status-badges">
        <div class="badge">
          <span id="dot-serial" class="badge-dot"></span>
          <span>底层串口 (<span id="txt-serial">检测中</span>)</span>
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
          <span style="font-size:0.8rem; color:var(--text-muted);">键盘 / 鼠标长按</span>
        </div>

        <!-- 调速与模式 -->
        <div class="controls-row">
          <div class="slider-group">
            <span style="font-size:0.85rem; color:var(--text-muted);">PWM速度:</span>
            <input type="range" id="pwm-slider" min="1000" max="3600" step="50" value="1800">
            <span id="pwm-val" class="slider-val" style="width:105px;">1800 (50%)</span>
          </div>
          <div style="display:flex; gap:6px; flex-wrap:wrap;">
            <button class="mode-tag" type="button" onclick="setPreset(1600)">🐢 慢速 1600 (45%)</button>
            <button class="mode-tag" type="button" onclick="setPreset(2200)">🚗 巡航 2200 (61%)</button>
            <button class="mode-tag" type="button" onclick="setPreset(2800)">⚡ 高速 2800 (78%)</button>
          </div>
          <div class="mode-tags" style="margin-top:4px;">
            <span class="mode-tag active" id="mode-hold">按住长按运行 (松手即停)</span>
            <span class="mode-tag" id="mode-pulse">点击点动微调 (1.2秒)</span>
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
          💡 <b>电脑键盘操控指南</b>: 支持 <kbd>W</kbd> 前进、<kbd>S</kbd> 后退、<kbd>A</kbd> 左转、<kbd>D</kbd> 右转，按住键行驶，松开自动刹停；任意时刻按 <kbd>空格键</kbd> 触发紧急制动。
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

        <!-- 视觉监控面板 -->
        <div class="card">
          <div class="card-title">
            <span>📸 IMX219 视觉抓拍回传</span>
            <span id="snap-time-tag" style="font-size:0.75rem; color:var(--text-muted);">暂无捕获</span>
          </div>
          <div class="camera-box">
            <div class="preview-frame" id="cam-frame">
              <span class="preview-placeholder" id="cam-placeholder">点击下方按钮抓拍图像</span>
              <img id="cam-img" alt="最新拍摄图片">
            </div>
            <button class="btn-snap" id="btn-snap">
              📸 拍摄高清照片 (1080P)
            </button>
          </div>
        </div>

        <!-- 控制事件日志 -->
        <div class="card" style="padding:14px;">
          <div class="card-title" style="padding-bottom:4px; font-size:0.9rem;">
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

    // DOM 元素
    const logBox = document.getElementById('log-box');
    const pwmSlider = document.getElementById('pwm-slider');
    const pwmVal = document.getElementById('pwm-val');
    const modeHold = document.getElementById('mode-hold');
    const modePulse = document.getElementById('mode-pulse');
    const btnStop = document.getElementById('btn-stop');
    const btnSnap = document.getElementById('btn-snap');
    const camImg = document.getElementById('cam-img');
    const camPlaceholder = document.getElementById('cam-placeholder');
    const snapTimeTag = document.getElementById('snap-time-tag');

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
      // 保持心跳防止服务端看门狗超时
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

    // 绑定 D-Pad 按钮事件 (兼容鼠标与触屏 Pointer Events)
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

    // 拍照功能
    btnSnap.addEventListener('click', async () => {
      btnSnap.disabled = true;
      btnSnap.textContent = "⏳ 正在拍摄处理中...";
      log("📸 发起摄像头抓拍请求...");
      try {
        const resp = await fetch('/api/snap', { method: 'POST' });
        const data = await resp.json();
        if (data.status === 'ok') {
          const imgUrl = '/api/image?t=' + new Date().getTime();
          camImg.src = imgUrl;
          camImg.style.display = 'block';
          camPlaceholder.style.display = 'none';
          snapTimeTag.textContent = "捕获于 " + (data.time || new Date().toLocaleTimeString());
          log("✓ 照片拍摄成功并加载到预览框");
        } else {
          log("❌ 拍照失败: " + data.message);
        }
      } catch (err) {
        log("❌ 抓拍网络请求出错: " + err);
      } finally {
        btnSnap.disabled = false;
        btnSnap.textContent = "📸 拍摄高清照片 (1080P)";
      }
    });

    // 传感器与状态轮询 (每 600ms)
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
        // 网络短暂断开静默处理
      }
    }

    setInterval(pollStatus, 600);
    pollStatus();
    log("🚀 控制台已就绪，已建立机载数据链路");
  </script>
</body>
</html>
"""


# ==========================================
# HTTP 服务处理器
# ==========================================
class CarHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # 屏蔽高频轮询的访问控制台刷屏日志
        if "/api/status" in args[0]:
            return
        super().log_message(format, *args)

    def do_GET(self):
        url_parts = urllib.parse.urlparse(self.path)
        path = url_parts.path

        if path == "/" or path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(INDEX_HTML.encode("utf-8"))

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
                "last_snap_time": hw.last_snap_time
            }
            self.wfile.write(json.dumps(status_data).encode("utf-8"))

        elif path == "/api/image":
            if os.path.exists(hw.last_snap_path):
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.end_headers()
                with open(hw.last_snap_path, "rb") as f:
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
                    # 脉冲移动：开动指定秒数后停止
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

        elif path == "/api/snap":
            success, info = hw.snap_photo()
            self.send_response(200 if success else 500)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            if success:
                res = {"status": "ok", "time": hw.last_snap_time}
            else:
                res = {"status": "error", "message": info}
            self.wfile.write(json.dumps(res).encode("utf-8"))
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

    server_address = ("0.0.0.0", port)
    httpd = ThreadingHTTPServer(server_address, CarHTTPHandler)

    print("=" * 60)
    print("   RBE201TC 智能小车机载 Web 控制中心已启动！")
    print(f"   本地服务端口: {port}")
    print(f"   电脑/手机浏览器访问: http://172.20.10.2:{port}")
    print("   按 Ctrl+C 安全停止退出")
    print("=" * 60)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n正在停止服务...")
    finally:
        hw.stop()
        httpd.server_close()
        print("服务已关闭，电机已急停")


if __name__ == "__main__":
    main()

