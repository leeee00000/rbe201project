# RBE201TC 智能小车系统全局索引与技术速查手册 (System Index & Dashboard)

本项目是专为 **地平线 RDK X5 (Linux Ubuntu 22.04 LTS, aarch64)** 平台设计的机器人集成系统，涵盖电机开环驱动、红外循迹导航、横向标记识别、高优先级异步温湿度采集、IMX219 高清视觉图传以及原子化数据持久化等软硬件实现。

系统追求**巡线循迹与环境数据采集的平稳连贯性**，本文档为新会话快速索引与高频速查卡。

---

## 一、 系统路径与源码架构索引

| 资源分类 | 物理路径 / 端口 | 说明 |
| :--- | :--- | :--- |
| **本地源码路径** | `E:\y3s1_learning\RBE201\file\Sources\RBE201TC\` | 宿主机开发与修改根目录 |
| **小车主板路径** | `/home/sunrise/RBE201TC/` | RDK X5 车端实际运行目录 |
| **数据日志路径** | `/home/sunrise/RBE201TC/temperature_points_<RunID>.csv` | 巡线环境采集原子化落盘日志 |
| **Web 控制台服务** | `http://192.168.43.30:8080` (无线热点) / `http://192.168.128.10:8080` (有线) | 手机/电脑免环境 B/S 遥控与实时图传 |

### 核心代码清单与功能分工

| 源码文件名 | 编程语言 | 核心作用与实现机制 |
| :--- | :--- | :--- |
| [`line_follower.py`](file:///e:/y3s1_learning/RBE201/file/Sources/RBE201TC/line_follower.py) | Python 3 | **[综合主控]** 50Hz 闭环巡线、横向地标防抖、脱轨自愈、异步温湿度采集与 CSV 记录 |
| [`web_controller.py`](file:///e:/y3s1_learning/RBE201/file/Sources/RBE201TC/web_controller.py) | Python 3 | **[机载Web]** 跨平台控制中心：按住/点动行驶、320x240 @ 18fps 低时延 MJPEG 双模图传 (彩色原图/赛道 Otsu 二值化一键切换)、探头指示灯与网页抓拍直存电脑本地 |
| [`terminal_console.py`](file:///e:/y3s1_learning/RBE201/file/Sources/RBE201TC/terminal_console.py) | Python 3 | **[终端控制]** 专为 TT 双线电机定制的 CLI 菜单控制台：1~8调速、微调、测探头与单轮排查 |
| `dht11_rdk_x5_gpio401.c` | C (GCC) | **[温湿度服务]** GPIO 401 bit-banging 微秒时序采集服务（支持标准 I/O IPC 管道通信） |
| `dht11_rdk_x5_test.c` | C (GCC) | **[传感器测试]** 单总线 C 语言独立周期测试程序 |
| [`camera_snap.py`](file:///e:/y3s1_learning/RBE201/file/Sources/RBE201TC/camera_snap.py) | Python 3 | **[视觉采集]** 调用地平线 `hobot_vio.libsrcampy` 捕获 1080P 帧并转为 BGR |
| [`line_sensor_test.py`](file:///e:/y3s1_learning/RBE201/file/Sources/RBE201TC/line_sensor_test.py) | Python 3 | **[探头测试]** 监听 4 路数字引脚，实时刷新电平与物理排列 |
| [`moto_test.py`](file:///e:/y3s1_learning/RBE201/file/Sources/RBE201TC/moto_test.py) | Python 3 | **[原始驱动测试]** 驱动板交互测试（注：针对 2 线 TT 电机请使用 `terminal_console.py`） |

---

## 二、 硬件元件与控制引脚速查卡

| 元件名称 | 硬件型号 / 芯片 | 物理接口 / 引脚 | 供电规范 | 控制与通信协议要点 |
| :--- | :--- | :--- | :--- | :--- |
| **四路驱动板 + TT电机** | Yahboom (STM32) + **4个双线 TT 减速电机 (纯开环)** | USB 串口 `/dev/ttyUSB0`<br>(115200, 8-N-1)；电机接 M1~M4 | **必须外接 12V 动力电池组**<br>(电池盒开关拨至 ON) | 开机配置：`$mtype:4#`<br>开环驱动：`$pwm:M1,M2,M3,M4#` (量程 `-3600~+3600`)<br>🛑 **严禁使用 `$spd:` 闭环指令** (无编码器会导致飞车) |
| **四路红外循迹模块** | TCRT5000 / 四路比较器板 | `GPIO.BOARD`：<br>X1:**Pin 15**, X2:**Pin 18**<br>X3:**Pin 16**, X4:**Pin 11** | 3.3V (Pin 1) 或 5V；GND: Pin 6 | 检测到黑线 / 悬空 = **`0`**<br>检测到白色底面 = **`1`** |
| **温湿度传感器** | DHT11 单总线模块 | DATA:**物理 Pin 37** (Linux **GPIO 401**)<br>VCC: 3.3V, GND: Pin 6 | 3.3V DC 稳压供电 | 单总线微秒级时序；后台 C 服务绑定 CPU Core 7 + SCHED_FIFO 50 实时调度 |
| **高清摄像头模块** | Sony IMX219 (800万) | 24-Pin FPC 排线插接 **CAM0**<br>I2C 控制：**i2c-6** (地址 **`0x10`**) | RDK X5 CSI 供电 | MIPI-CSI-2 传输；地平线 VIO 硬件加速管道 |

---

## 三、 高频常用运行命令

```bash
# 1. 登录 RDK X5 车载终端 (密码: sunrise)
ssh sunrise@192.168.43.30       # 当前个人手机热点无线连 (固定租约)
ssh sunrise@192.168.128.10      # Type-C 数据线直连 (板载固定静态 IP)
# 历史组员热点备用: ssh sunrise@172.20.10.2

# 2. 编译温湿度采集 C 服务 (进入板端工作目录)
cd ~/RBE201TC && gcc dht11_rdk_x5_gpio401.c -o dht11_rdk_x5_gpio401_server -Wall && chmod +x dht11_rdk_x5_gpio401_server

# 3. 运行终端极客控制台 (调参、单电机通道排查、探头与拍照)
python3 ~/RBE201TC/terminal_console.py

# 4. 运行机载 Web 控制中心 (电脑/手机浏览器打开 http://172.20.10.2:8080)
python3 ~/RBE201TC/web_controller.py

# 5. 启动综合自主巡线主程序 (按 Ctrl+C 安全刹停)
sudo python3 ~/RBE201TC/line_follower.py
```

---

## 四、 详细专题分册与扩展文档指引

遇到深入的技术调试、参数调优或历史排障，请查阅对应专题分册：

- 👶 **[面向新手的极简保姆级教程]**：[readme0.md](file:///e:/y3s1_learning/RBE201/file/Sources/readme0.md)（小学生也能看懂的小车控制与安全启动说明）
- 🔌 **[引脚与电气规范分册]**：[docs/01_hardware_pinouts.md](file:///e:/y3s1_learning/RBE201/file/Sources/docs/01_hardware_pinouts.md)
- ⚙️ **[电机物理与串口协议分册]**：[docs/02_motor_protocol.md](file:///e:/y3s1_learning/RBE201/file/Sources/docs/02_motor_protocol.md)（飞车暴冲与死区数学机理解析）
- 📡 **[传感器与视觉模块分册]**：[docs/03_sensor_modules.md](file:///e:/y3s1_learning/RBE201/file/Sources/docs/03_sensor_modules.md)（加权巡线、DHT11微秒时序与CSI底层）
- 🚗 **[巡线状态机与CSV日志规范]**：[docs/04_line_follower_tuning.md](file:///e:/y3s1_learning/RBE201/file/Sources/docs/04_line_follower_tuning.md)
- 🛠️ **[故障排查与安全守则分册]**：[docs/05_troubleshooting.md](file:///e:/y3s1_learning/RBE201/file/Sources/docs/05_troubleshooting.md)
- 📋 **[实验课实操与连接全指南]**：[docs/plan.md](file:///e:/y3s1_learning/RBE201/file/Sources/docs/plan.md)（VS Code Remote-SSH、代码互传与五阶段清单）
- 📓 **[实验室实操与测试日志]**：[docs/lab_log.md](file:///e:/y3s1_learning/RBE201/file/Sources/docs/lab_log.md)（历史测试结果、踩坑记录与下次测试待办清单）
