# 传感器与视觉模块技术指南 (Sensors & Vision Modules)

本文档整理了红外循迹、温湿度检测和高清摄像头三大感知模块的控制逻辑、底层驱动实现与测试流程。

---

## 一、 四路红外循迹传感器 (Line Tracking Sensors)

### 1. 逻辑特性与加权算法
- **输入电平**：
  - 黑线 / 悬空（无反射）：读取为 **`0`**；
  - 白底（强反射）：读取为 **`1`**。
- **位置加权公式**：
  设探头从左至右物理权重依次为：
  $$W_{X1} = -3.0,\quad W_{X2} = -1.0,\quad W_{X3} = +1.0,\quad W_{X4} = +3.0$$
  偏差误差计算公式：
  $$\text{Error} = \frac{\sum_{i \in \text{黑线探头}} W_i}{N_{\text{黑线探头数}}}$$
  - $\text{Error} \in [-0.25, 0.25]$：车辆居中；
  - $\text{Error} < 0$：车体偏右（线在左侧），需向左纠偏；
  - $\text{Error} > 0$：车体偏左（线在右侧），需向右纠偏；
  - 4 路全为 0：触发横向全黑标记线（检测点）；
  - 4 路全为 1：判定为脱轨（Lost-line）。

### 2. 独立测试流程
1. 运行监听程序：
   ```bash
   sudo python3 ~/RBE201TC/line_sensor_test.py
   ```
2. 拿白纸置于探头下方（约 1~1.5cm），终端应显示 `X1:1  X2:1  X3:1  X4:1`；
3. 手指或黑色绝缘胶带从左到右依次遮挡，终端应依次变为 `0`：
   $$\text{左} \implies X1(15) \to X2(18) \to X3(16) \to X4(11) \implies \text{右}$$
4. 调节模块上的可调电位器，直至探头指示灯在离开白纸时灵敏熄灭。

---

## 二、 DHT11 温湿度传感器 (Temperature & Humidity Sensor)

### 1. 底层通信时序与进程间通信 (IPC) 协议
- **单总线时序**：主机拉低 GPIO 401 至少 18ms，拉高 20~40μs，等待 DHT11 响应低电平 80μs 和高电平 80μs；随后读取 40 位数据（高电平持续 26~28μs 为 `0`，70μs 为 `1`），最后 8 位为前 4 字节和校验。
- **C 服务后台 IPC 协议 (`dht11_rdk_x5_gpio401_server --server`)**：
  - 主控 Python 发送命令（通过子进程标准输入 stdin）：
    ```text
    READ <point_id>\n
    ```
  - C 服务标准输出 (stdout) 回应：
    ```text
    DHT\t<point_id>\tOK\t<湿度整数.小数>\t<温度整数.小数>\n     # 成功
    DHT\t<point_id>\tERROR\t<错误描述文本>\n                     # 失败
    ```
  - 退出命令：发送 `QUIT\n`。
- **高实时性调度配置**：
  主控程序使用以下系统命令启动服务，避免 Linux 调度延迟破坏微秒时序：
  ```bash
  chrt -f 50 taskset -c 7 ./dht11_rdk_x5_gpio401_server --server
  ```
  - `taskset -c 7`：绑定至独立 CPU Core 7；
  - `chrt -f 50`：赋予 SCHED_FIFO 实时调度优先级 50。

### 2. 测试流程
1. 编译 C 语言程序：
   ```bash
   cd ~/RBE201TC
   gcc dht11_rdk_x5_test.c -o dht11_test -Wall
   gcc dht11_rdk_x5_gpio401.c -o dht11_rdk_x5_gpio401_server -Wall
   chmod +x dht11_rdk_x5_gpio401_server
   ```
2. 独立测试：
   ```bash
   sudo ./dht11_test
   ```
   正常应每隔 2 秒打印一次温湿度与成功率：
   `✓ humidity：55.0 %RH，temperature：24.0 °C，Success/Failure：1/0`。

---

## 三、 索尼 IMX219 高清摄像头 (MIPI-CSI Camera)

### 1. 底层驱动与编程接口
- **总线寻址**：摄像头通过 24-Pin FPC 排线接入 CAM0，I2C 控制挂载在 `i2c-6`，I2C 芯片设备地址为 **`0x10`**。
- **Python 硬件加速调用库**：`from hobot_vio import libsrcampy as srcampy`。
- **基本控制代码范式**：
  ```python
  from hobot_vio import libsrcampy as srcampy
  import cv2, numpy as np

  cam = srcampy.Camera()
  # 开启 1080P/30fps 视频流: open_cam(pipe_id, video_idx, fps, width_list, height_list, sensor_h, sensor_w)
  cam.open_cam(0, -1, 30, [1920], [1088], 1080, 1920)

  # 获取 NV12 格式图像帧
  img_bytes = cam.get_img(2, 1920, 1088)

  # NV12 转 BGR
  nv12_arr = np.frombuffer(img_bytes, dtype=np.uint8).reshape((1088 * 3 // 2, 1920))
  bgr_img = cv2.cvtColor(nv12_arr, cv2.COLOR_YUV2BGR_NV12)[:1080, :1920]

  # 保存图片或进行 OpenCV 视觉处理
  cv2.imwrite("frame.jpg", bgr_img)
  cam.close_cam()
  ```

### 2. 测试流程
1. 确认断电状态下插好排线，开机；
2. 运行一键拍照测试脚本：
   ```bash
   python3 ~/RBE201TC/camera_snap.py
   ```
3. 程序自动捕获 1080P 高清画面并保存为 `test_camera.jpg`，确认视觉通路正常。

---

## 四、 机载实时双模式视频流与前端本地下载架构 (`web_controller.py`)

在 `web_controller.py` 中，视觉模块已升级为常驻低延迟图传引擎 `CameraStreamer`，专为赛道巡线调试与低带宽无线热点优化：

### 1. 核心架构特性
1. **多线程生产者-消费者解耦**：
   - 采集线程以 `18fps` 恒定节流，避免过多占用 RDK X5 核心 CPU；
   - 视频帧通过全局线程锁单帧覆盖，HTTP 客户端断开连接时通过捕获 `BrokenPipeError` 优雅退出，防止 CSI 硬件通道死锁。
2. **极速赛道二值化算力优化 (Otsu)**：
   - **NV12 Y-分量直取**：IMX219 捕获的 NV12 格式中前 $1920 \times 1080$ 字节即为亮度 Y 通道（纯灰度数据），代码直接截取该区域进行高斯滤波与 Otsu 大津二值化，**省去了 CPU 执行 `cvtColor(NV12 -> BGR)` 的高额开销**；
   - **分辨率降采样**：图像下采样至 `320x240`，JPEG 压缩品质设为 75，单帧体积仅 `8~12KB`，网络传输延迟 $< 100\text{ms}$。
3. **网页端双模式无缝切换**：
   - 网页提供 `GET /video_feed`（持续推流）与 `POST /api/camera_mode`（模式切换）；
   - 用户在浏览器点击【🌈 彩色原图】与【🏁 赛道二值化】即可毫秒级平滑切换，用于直观检验赛道黑线反差与光影抗干扰情况。
4. **前端直存电脑本地下载机制**：
   - 网页端抓拍放弃了传统的“仅存小车再用 scp 拉取”模式，采用 **HTML5 Canvas 截取当前视频帧 $\to$ 转为二进制 Blob 对象 $\to$ 创建虚拟 `<a>` 标签触发下载**；
   - 照片以 `snap_YYYYMMDD_HHMMSS.jpg` 自动命名，点击直接保存到操作者电脑的本地“下载 (Downloads)”目录中。

