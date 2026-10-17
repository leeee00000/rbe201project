# RBE201TC 智能小车实验课实操与连接全指南 (Lab Plan)

**实验日期**：2026-10-09（明天实验课）  
**主板平台**：地平线 RDK X5 (Ubuntu 22.04.5 LTS, Python 3.10.12)  
**默认凭据**：用户名 `sunrise` | 默认密码 `sunrise`  
**核心任务**：VS Code Remote-SSH 图形化连接、代码/数据一键互传、电机/转向测试、循迹传感器物理标定、巡线自主导航（寻路）实地调优、温湿度数据记录。

---

## 一、 VS Code Remote - SSH 一键图形化连接指南（推荐工作流 ⭐⭐⭐）

使用 VS Code 官方的 **Remote - SSH** 插件连接是最直观、最高效的方式：可以在电脑上**直接图形化查看/修改小车代码、直接拖拽传文件、右键一键下载 CSV 数据、内置终端无缝执行 Linux 命令**。

### 1. 本地已为您配置好的两套连接通道
您的本地配置文件（`C:\Users\leo\.ssh\config`）已自动配置完毕，包含两种场景：
- **`RDK_X5_USB`**：`192.168.127.10`（Type-C 数据线直连电脑，**无需 Wi-Fi，最稳保底**）
- **`RDK_X5`**：`172.20.10.2`（手机热点 / Wi-Fi 无线连接，**小车下地跑动时用**）

---

### 2. VS Code 一键连接 3 步走

```mermaid
flowchart LR
    A["1. 点击左侧活动栏<br/>Remote Explorer<br/>(远程资源管理器图标)"] --> B["2. 在 SSH 列表找到<br/>RDK_X5_USB 或 RDK_X5<br/>点击'在新窗口连接'箭头"] --> C["3. 提示输入密码<br/>输入: sunrise<br/>(回车完成连接)"]
```

1. **进入远程视图**：点击 VS Code 最左侧活动栏的 **Remote Explorer（远程资源管理器）** 图标（显示为一个小电脑屏幕）；
2. **点击连接**：在 SSH Targets 列表中会看到 `RDK_X5_USB` 和 `RDK_X5`：
   - 调试时插着 Type-C 线：点击 **`RDK_X5_USB`** 右侧的箭头（Connect to Host in New Window）；
   - 实车跑线无连线状态：小车连手机热点后，点击 **`RDK_X5`** 右侧的箭头；
3. **输入密码**：上方弹窗如果提示选择操作系统，选择 **`Linux`**；提示输入密码时输入 **`sunrise`** 按回车，窗口左下角显示 `SSH: RDK_X5...` 即表示连接成功！

---

### 3. 现场 IP 变动时的 10 秒修改法（若手机热点分配了新 IP）
如果在实验室连了新热点，小车 IP 变了，无需重新配插件：
1. 在 VS Code 中按下快捷键 **`F1`** 或 **`Ctrl + Shift + P`**；
2. 输入并选择：`Remote-SSH: Open SSH Configuration File...`（打开 SSH 配置文件）；
3. 点击选择 `C:\Users\leo\.ssh\config`；
4. 将对应 `HostName` 后面的 IP 修改为小车当前的新 IP（例如改为 `172.20.10.3`），按 `Ctrl + S` 保存；
5. 回到 Remote Explorer，点击刷新或直接重新连接即可！

---

### 4. 连接成功后的“神级操作”技巧（免打复杂命令）

- **① 打开小车代码文件夹**：
  - 点击 VS Code 菜单栏 `文件 (File)` $\to$ `打开文件夹 (Open Folder)`；
  - 路径输入框输入：`/home/sunrise/RBE201TC`（或 `/home/sunrise`），点击确定并再次输入密码 `sunrise`。
  - **效果**：左侧文件树直接展示小车里的全部 Python 和 C 代码，双击即可高亮查看与修改！
- **② 调出远程 Linux 终端**：
  - 按快捷键 **`Ctrl + ~`**（Esc 下方的反引号键）；
  - 底部直接呼出小车的命令行终端：`sunrise@ubuntu:~/RBE201TC$`，后续所有测试命令直接在此处粘贴运行！
- **③ 极速上传本地代码（拖拽即传）**：
  - 如果发现主板里没有代码，直接从 Windows 文件管理器中把本机的 `RBE201TC` 文件夹**拖入** VS Code 左侧的文件树空白区域，自动上传！
- **④ 课后一键下载 CSV 数据（右键另存）**：
  - 小车跑完巡线后，在 VS Code 左侧文件树中找到生成的 `temperature_points_xxx.csv`；
  - **鼠标右键点击该文件 $\to$ 选择“下载... (Download...)”**，直接保存到 Windows 本地，完全不需要敲复杂的 SCP 命令！

---

### 5. 备用通道：原生 PowerShell 命令行直连
若遇 VS Code 插件偶尔卡顿，可随时在 Windows PowerShell 中备用连接：
```powershell
# Type-C 直连
ssh sunrise@192.168.127.10

# 手机热点无线连
ssh sunrise@172.20.10.2
```

---

## 二、 课前安全防范守则（核心必读 ⚠️）

> [!CAUTION]
> 1. **严禁带轮着地直接上电运行**：在进行电机测试和初次调参前，**必须用支架或水杯将小车底盘垫高，使四个驱动轮完全悬空**，防止程序失控“飞车”跌落摔坏 RDK X5 核心板。
> 2. **电源开关随手可及**：始终清楚电机独立供电开关的位置，若遇到无法软件急停的情况，第一时间手动切断电源。
> 3. **接线防短路**：插拔杜邦线和 USB 线时务必先断电。

---

## 三、 阶段 0：环境自检与 C 服务编译 (约 5 分钟)

在 VS Code 内置终端（`Ctrl + ~`）中执行：

### 1. 检查代码存在性与 ROS2 影响排查
```bash
# (1) 检查代码是否在主板中
ls -la ~/RBE201TC

# (2) 检查磁盘剩余空间（确保根分区 / 可用空间大于 1GB）
df -h

# (3) 检查 ~/.bashrc 是否有重复 source ROS 环境变量
tail -n 20 ~/.bashrc
# 如果看到多行 source /opt/ros/.../setup.bash，用 nano ~/.bashrc 删掉重复行，只留一行

# (4) 验证实验所需的底层硬件库是否健全
python3 -c "import serial; import Hobot.GPIO; print('✓ 核心硬件库正常')"
command -v gcc chrt taskset
```

### 2. 编译 DHT11 采集服务
```bash
cd ~/RBE201TC

# 检查串口设备（若出现 /dev/ttyUSB0 表明电机驱动板连接正常）
ls -l /dev/ttyUSB* /dev/ttyACM*

# 编译 C 语言温湿度采集服务（为后续综合运行做准备）
gcc dht11_rdk_x5_test.c -o dht11_test -Wall
gcc dht11_rdk_x5_gpio401.c -o dht11_rdk_x5_gpio401_server -Wall
chmod +x dht11_rdk_x5_gpio401_server
```

---

## 四、 阶段 1：电机驱动与转向控制测试 (约 15~20 分钟)

**工具脚本**：`terminal_console.py`（推荐，专为 TT 双线电机开环 PWM 定制）或 `moto_test.py`  
**测试状态**：**车轮悬空离地**。

在终端中运行：
```bash
cd ~/RBE201TC
python3 terminal_console.py
```

### 核心检验清单：
- [ ] **步骤 1.1：单电机逐一测试（选择 Option 2）**
  - 分别测试通道 1、2、3、4（转速默认 300，运行 3 秒）：
    - 记录通道与车轮位置对应关系：确认 **左侧电机为 M1、M2**，**右侧电机为 M3、M4**。
    - 观察车轮正转方向：正数速度下车轮必须向前旋转。若反向旋转，记录需调换方向的通道。
- [ ] **步骤 1.2：差速与车体动作测试（选择 Option 3）**
  - 观察原地旋转（左侧前转、右侧反转）和小车前后直行是否协调。
- [ ] **步骤 1.3：急停与中断安全测试**
  - 在手动控制（Option 4）或基础运行中，随时按下 `Ctrl+C` 或输入 `s`，验证电机是否能够立即完全锁死刹停。

---

## 五、 阶段 2：四路循迹传感器物理标定 (约 10~15 分钟)

**工具脚本**：`line_sensor_test.py`  
**测试目的**：确认探头排列物理顺序、黑白反射电平逻辑与高度灵敏度。

```bash
cd ~/RBE201TC
sudo python3 line_sensor_test.py
```

### 核心检验清单：
- [ ] **步骤 2.1：高低电平响应校验**
  - 将传感器置于白色底板上，观察终端读数（应为 `1`）；
  - 用黑色绝缘胶带置于探头下方，观察终端读数（应变为 `0`）。
  - *注意：若反转，需确认代码中的 `BLACK_LEVEL = 0` 配置是否需要调整。*
- [ ] **步骤 2.2：物理左右顺序核对**
  - 手指从**左到右**依次遮挡探头，终端输出变化的通道必须严格对应：
    $$\text{从左到右} \implies \text{X1 (Pin 15)} \to \text{X2 (Pin 18)} \to \text{X3 (Pin 16)} \to \text{X4 (Pin 11)}$$
  - 若顺序错乱，小车循迹时会反向冲出赛道！
- [ ] **步骤 2.3：探头离地高度与灵敏度微调**
  - 调节探头模块上的微调电位器，确保在实际场地反光光照下，探头离地约 1~2cm 时能清晰灵敏地区分黑线与白地。

---

## 六、 阶段 3：巡线寻路（自主导航）实车联调 (约 30~45 分钟)

**核心程序**：`line_follower.py`  
**初始参数设置**（直接在 VS Code 编辑器中打开 `line_follower.py` 查看或修改）：
- 基础直行速度：`BASE_PWM = 220`
- 最大限幅速度：`MAX_PWM = 300`
- 缓弯内轮/外轮速度：`SOFT_INNER_PWM = 130`, `SOFT_OUTER_PWM = 220`
- 急弯差速原地自转速度：`HARD_TURN_PWM = 210`

### 1. 初次着地测试 (First Run)
小车断开 Type-C 线，连上手机热点（在 VS Code 中通过 `RDK_X5` 无线连接观察）：
```bash
sudo python3 line_follower.py
```

### 2. 调优要点与现象对策表

| 运行现象 | 可能原因 | 解决对策 (在 VS Code 中修改 `line_follower.py`) |
| :--- | :--- | :--- |
| **小车站立不动或电机发出蜂鸣声** | 驱动 PWM 低于死区电压 | 适当调高 `BASE_PWM`（如提升到 240~260）。 |
| **直道行驶剧烈“画龙”蛇形摆动** | 纠偏反应过猛或内侧轮减速过大 | 增大 `SOFT_INNER_PWM`（缩小内外轮差速），或适当增加 `RECENTER_BRAKE_SECONDS`（回中制动缓冲）。 |
| **直角弯或急弯冲出黑线脱轨** | 转向力矩不足或循迹频率偏低 | 调大 `HARD_TURN_PWM`（如提升至 230~250），适当降低直道速度 `BASE_PWM`。 |
| **遇到脱轨后无法找回黑线** | 脱线后旋转搜索方向错误 | 检查 `calculate_error` 探头左右权重是否与硬件实际一致。 |

### 3. 脱轨保护自愈功能验证
- 在小车运行中故意移开黑线：
  - 观察小车是否执行：**点刹 (0.06s) $\to$ 短暂后退 (0.10s) $\to$ 原地搜索 $\to$ 5秒超时安全锁死**。

---

## 七、 阶段 4：横向标记点与温湿度记录联动 (约 15~20 分钟)

若场地布置了横向黑胶带标记线（检测点）：
1. 观察小车匀速穿过横向全黑胶带时，四路传感器全为 0 是否触发标记点；
2. 确认小车**不停车平稳穿过**，VS Code 终端实时输出：`📍 Detection point X: trigger recorded; asynchronous reading requested.`；
3. 随后观察是否输出采集成功日志：`🌡 Detection point X: humidity ...%RH, temperature ...°C. Saved`；
4. 按 `Ctrl+C` 结束程序，左侧文件树刷新即可看到新生成的 `temperature_points_xxx.csv`。

---

## 八、 阶段 5：课后收尾与数据归档 (约 5 分钟)

1. **一键下载 CSV 数据**：
   - 在 VS Code 左侧文件树中找到 `temperature_points_*.csv`；
   - **鼠标右键 $\to$ 点击“下载 (Download...)”** $\to$ 保存到本地电脑 `E:\y3s1_learning\RBE201\file\Sources\`。
2. **记录参数配置与安全关机**：
   - 在代码注释中记录今天跑通赛道的最佳 PWM 参数组合；
   - 在 VS Code 终端中安全关机：
     ```bash
     sudo poweroff
     ```
     等待主板绿灯熄灭后拔掉电源。

