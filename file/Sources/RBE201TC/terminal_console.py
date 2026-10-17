#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RBE201TC 智能小车简易终端控制台 (Terminal Console)
专为 2 线双极性普通直流减速电机定制 (无编码器，开环 PWM，满量程 0~3600)
日志自动持久化至: /home/sunrise/RBE201TC/terminal_console.log
"""

import os
import sys
import time
import logging
import serial

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
# 日志配置
# ==========================================
LOG_FILE_PATH = os.path.expanduser("/home/sunrise/RBE201TC/terminal_console.log")
if os.name == "nt":
    LOG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "terminal_console.log")

logger = logging.getLogger("CarTerminal")
logger.setLevel(logging.DEBUG)
if logger.hasHandlers():
    logger.handlers.clear()

file_handler = logging.FileHandler(LOG_FILE_PATH, mode="a", encoding="utf-8")
file_handler.setLevel(logging.DEBUG)
file_formatter = logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
file_handler.setFormatter(file_formatter)
logger.addHandler(file_handler)

console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)
console_formatter = logging.Formatter("%(message)s")
console_handler.setFormatter(console_formatter)
logger.addHandler(console_handler)


class SimpleCarController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
        
        # Yahboom 驱动板 PWM 满量程为 3600 (0~3600)
        # 1800 为 50% 占空比 (约 6V)，适合平稳低速空转测试
        self.default_pwm = 1800
        self.tracking_pins = {"X1": 15, "X2": 18, "X3": 16, "X4": 11}

        logger.info(f"\n=======================================================")
        logger.info(f"🚀 控制台启动 | TT 无编码器电机模式 (mtype:4, PWM 满量程 3600)")
        logger.info(f"📁 日志路径: {LOG_FILE_PATH}")
        logger.info(f"=======================================================")

        self._init_serial()
        self._init_gpio()

    def _init_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.15)
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            logger.info(f"✓ 串口已连接: {self.port} @ {self.baudrate}")

            # 驱动板硬件复位与针对无编码器 TT 电机的正确模式配置
            self.send_cmd("$mtype:4#")        # 激活 TT 电机无编码器模式 (非常关键！)
            self.send_cmd("$upload:0,0,0#")    # 禁用编码器回传流
            self.send_cmd("$pwm:0,0,0,0#")     # 初始占空比置零
            logger.info("✓ 驱动板初始化完成 (已激活 mtype:4 无编码器电机模式)")
        except Exception as e:
            logger.error(f"⚠ 串口打开失败 ({self.port}): {e}")
            self.connected = False

    def _init_gpio(self):
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BOARD)
                for pin in self.tracking_pins.values():
                    GPIO.setup(pin, GPIO.IN)
                logger.info("✓ 循迹传感器 GPIO 就绪 (BOARD 模式)")
            except Exception as e:
                logger.warning(f"⚠ GPIO 初始化异常: {e}")

    def send_cmd(self, cmd):
        """发送底层 ASCII 串口指令"""
        if not self.connected:
            logger.error(f"❌ 串口未连接: {cmd}")
            return False
        try:
            self.ser.write(cmd.encode("utf-8"))
            self.ser.flush()
            logger.debug(f"[TX] 发送指令: {cmd}")
            return True
        except Exception as e:
            logger.error(f"❌ 写入失败 ({cmd}): {e}")
            return False

    def send_pwm(self, m1, m2, m3, m4, action_name="运动"):
        # Yahboom 占空比钳位在 -3600 ~ +3600
        m1 = int(max(-3600, min(3600, m1)))
        m2 = int(max(-3600, min(3600, m2)))
        m3 = int(max(-3600, min(3600, m3)))
        m4 = int(max(-3600, min(3600, m4)))
        cmd = f"$pwm:{m1},{m2},{m3},{m4}#"
        pct = round(abs(m1) / 3600 * 100)
        logger.info(f">>> 🚗 {action_name} | PWM值: ({m1}, {m2}, {m3}, {m4}) [约 {pct}% 功率]")
        self.send_cmd(cmd)

    def forward(self, duration=None, pwm=None):
        val = pwm if pwm is not None else self.default_pwm
        self.send_pwm(val, val, val, val, action_name=f"前进")
        if duration:
            time.sleep(duration)
            self.stop()

    def backward(self, duration=None, pwm=None):
        val = pwm if pwm is not None else self.default_pwm
        self.send_pwm(-val, -val, -val, -val, action_name=f"后退")
        if duration:
            time.sleep(duration)
            self.stop()

    def turn_left(self, duration=None, pwm=None):
        val = pwm if pwm is not None else self.default_pwm
        self.send_pwm(-val, -val, val, val, action_name=f"原地左转")
        if duration:
            time.sleep(duration)
            self.stop()

    def turn_right(self, duration=None, pwm=None):
        val = pwm if pwm is not None else self.default_pwm
        self.send_pwm(val, val, -val, -val, action_name=f"原地右转")
        if duration:
            time.sleep(duration)
            self.stop()

    def test_single_motor(self, channel, val=None):
        """单独测试某一个通道通道 M1~M4"""
        speed = val if val is not None else self.default_pwm
        m = [0, 0, 0, 0]
        m[channel - 1] = speed
        logger.info(f"\n--- 单独驱动通道 M{channel} (PWM {speed}) 持续 2 秒 ---")
        self.send_pwm(m[0], m[1], m[2], m[3], action_name=f"单轮通道M{channel}")
        time.sleep(2.0)
        self.stop()

    def stop(self):
        logger.info(">>> 🛑 安全刹车停转")
        self.send_cmd("$pwm:0,0,0,0#")

    def read_tracking(self):
        if not GPIO_AVAILABLE:
            logger.warning("⚠ Hobot.GPIO 模块不可用")
            return
        logger.info("\n=== 📡 四路红外循迹传感器实时读取 (黑0白1) ===")
        for i in range(10):
            res_dict = {k: GPIO.input(p) for k, p in self.tracking_pins.items()}
            res_str = "   ".join([f"{k}:{v}" for k, v in res_dict.items()])
            logger.info(f"[{i+1}/10]  {res_str}")
            time.sleep(0.2)

    def snap_photo(self, output_path="/home/sunrise/RBE201TC/test_camera.jpg"):
        if not CAM_AVAILABLE:
            logger.warning("⚠ 摄像头相关依赖库不可用")
            return
        logger.info("\n>>> 📸 正在调用 IMX219 摄像头拍摄...")
        try:
            cam = srcampy.Camera()
            ret = cam.open_cam(0, -1, 30, [1920], [1088], 1080, 1920)
            if ret != 0:
                logger.error(f"❌ 摄像头开启失败: {ret}")
                return
            time.sleep(0.8)
            img = cam.get_img(2, 1920, 1088)
            if img is not None:
                arr = np.frombuffer(img, dtype=np.uint8).reshape((1088 * 3 // 2, 1920))
                bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_NV12)[:1080, :1920]
                cv2.imwrite(output_path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                logger.info(f"🎉 拍照成功！已保存到: {output_path}")
            cam.close_cam()
        except Exception as e:
            logger.error(f"❌ 拍照出错: {e}")

    def close(self):
        self.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        logger.info("控制器已安全释放退出")


def print_menu(car):
    pct = round(car.default_pwm / 3600 * 100)
    print("\n" + "=" * 60)
    print("      RBE201TC 智能小车简易终端控制台 (开环 PWM 调速)")
    print(f"  当前设定 PWM: {car.default_pwm} / 3600 ({pct}% 功率)")
    print("  [参考: 1600~1800 平稳空转 | 2000~2400 落地巡航 | 2800+ 强劲高速]")
    print(f"  日志持久化文件: {LOG_FILE_PATH}")
    print("=" * 60)
    print("  [1] 前进 (持续前进)")
    print("  [2] 后退 (持续后退)")
    print("  [3] 原地左转")
    print("  [4] 原地右转")
    print("  [0] 🛑 立即急停刹车 (或直接按空格键)")
    print("  --------------------------------------")
    print("  [w] 脉冲前进 1.5 秒 (自动刹停)")
    print("  [s] 脉冲后退 1.5 秒 (自动刹停)")
    print("  [a] 脉冲左转 1 秒")
    print("  [d] 脉冲右转 1 秒")
    print("  --------------------------------------")
    print("  [7] ⚙️ 设定 PWM 数值 (满量程 0~3600，建议 1500~2500)")
    print("  [t] 单轮单独排查测试 (M1~M4 逐轮测试 2 秒)")
    print("  [5] 📡 读取四路循迹探头 (X1~X4) 实时状态")
    print("  [6] 📸 触发 IMX219 摄像头拍摄一张照片")
    print("  [8] 📜 查看最近 20 行运行调试日志")
    print("  [q] 退出控制台")
    print("=" * 60)


def view_logs(tail_lines=20):
    print(f"\n--- 📜 最近 {tail_lines} 行调试日志 ---")
    if not os.path.exists(LOG_FILE_PATH):
        print("暂无日志记录")
        return
    try:
        with open(LOG_FILE_PATH, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
            for line in lines[-tail_lines:]:
                print(line.rstrip())
    except Exception as e:
        print(f"读取日志失败: {e}")
    print("--- 日志结束 ---\n")


def main():
    car = SimpleCarController()
    try:
        while True:
            print_menu(car)
            choice = input("请输入控制指令编号并回车: ").strip().lower()
            logger.debug(f"[USER_INPUT] 输入指令: '{choice}'")

            if choice == "1":
                car.forward()
            elif choice == "2":
                car.backward()
            elif choice == "3":
                car.turn_left()
            elif choice == "4":
                car.turn_right()
            elif choice in ("0", " ", "stop"):
                car.stop()
            elif choice == "w":
                car.forward(duration=1.5)
            elif choice == "s":
                car.backward(duration=1.5)
            elif choice == "a":
                car.turn_left(duration=1.0)
            elif choice == "d":
                car.turn_right(duration=1.0)
            elif choice == "7":
                try:
                    new_val = int(input("请输入新的 PWM 数值 (满量程 0~3600，平稳慢速推荐 1600~1800): ").strip())
                    car.default_pwm = max(500, min(3600, new_val))
                    logger.info(f"✓ 默认 PWM 已调整为: {car.default_pwm}")
                except ValueError:
                    logger.warning("❌ 请输入有效整数")
            elif choice == "t":
                ch_str = input("请输入要测试的电机通道编号 (1=左前, 2=左后, 3=右前, 4=右后): ").strip()
                if ch_str in ("1", "2", "3", "4"):
                    car.test_single_motor(int(ch_str))
                else:
                    print("⚠ 请输入 1~4")
            elif choice == "5":
                car.read_tracking()
            elif choice == "6":
                car.snap_photo()
            elif choice == "8":
                view_logs(20)
            elif choice in ("q", "quit", "exit"):
                break
            else:
                logger.warning("⚠ 未知选项，请重新选择")
    except KeyboardInterrupt:
        logger.info("\n\n⚠ 检测到中断信号...")
    finally:
        car.close()


if __name__ == "__main__":
    main()
