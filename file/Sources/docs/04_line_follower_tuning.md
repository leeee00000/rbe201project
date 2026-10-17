# 综合集成主控程序与调参指南 (Line Follower Tuning & State Machine)

主控程序 `line_follower.py` 整合了全部硬件组件，核心设计目标为：**连续巡线导航、横向标记防抖识别、不停车异步温湿度采集、原子化安全写入 CSV**。系统重在保证巡线与数据采集过程的连贯性与稳定可靠性。

---

## 一、 核心控制参数设定

| 参数宏定义 | 默认值 | 作用与物理意义 |
| :--- | :---: | :--- |
| `BASE_PWM` | 220 | 直道基础巡航 PWM 占空比 |
| `MAX_PWM` | 300 | 限制的最大安全 PWM 占空比 |
| `SOFT_INNER_PWM` / `SOFT_OUTER_PWM` | 130 / 220 | 浅弯小差速纠偏速度 |
| `MEDIUM_INNER_PWM` / `MEDIUM_OUTER_PWM` | 0 / 220 | 明显偏航时内侧停转差速 |
| `HARD_TURN_PWM` | 210 | 严重偏航时差速原地自转速度 |
| `CONTROL_HZ` | 50.0 | 主控状态机循环更新频率 (50Hz = 每周期 20ms) |
| `RECENTER_BRAKE_SECONDS` | 0.08 | 车辆从转弯回归中线时的主动点刹缓冲时间 (消除直道蛇形摇摆) |
| `MARKER_CONFIRM_SECONDS` | 0.04 | 四路全黑持续判定时间阈值 (确认有效横向标记，消除杂色误触发) |
| `MARKER_CLEAR_SECONDS` | 0.10 | 离开横向黑线所需的白底恢复时间 (解除标记锁存，防重复计数) |
| `DHT_REQUEST_TIMEOUT_SECONDS` | 10.0 | 异步温湿度读取的最长超时判定时间 |

> [!NOTE]
> **主控巡线 PWM 量程适配说明**：
> 在原始 `line_follower.py` 模板中，设定了 `BASE_PWM = 220`、`MAX_PWM = 300` 以及 `MOTOR_DEADZONE = 1900`（默认 `CONFIGURE_DEADZONE_ON_START = False` 未向单片机写入死区参数）。
> 由于 Yahboom 驱动板针对 TT 电机（`mtype:4`）的满量程为 **3600**（死区约 1200~1600），在实车落地测试时：
> - 若使用未修改的 `line_follower.py` 且发现电机扭矩偏小或不转，只需将 `line_follower.py` 开头的 `MOTOR_TYPE = 1` 改为 `MOTOR_TYPE = 4`，并将 `BASE_PWM` 按 3600 满量程放大至 **`2000 ~ 2400`**，`MAX_PWM` 设为 **`3200`** 即可平顺巡线。

---

## 二、 状态机行为逻辑

1. **中线直行**：误差在 $\pm 0.25$ 范围内，执行 `BASE_PWM` 直行；刚转弯回中时触发短点刹；
2. **分级转向**：根据加权误差绝对值分级施加差速，极端偏角下驱动轮执行正反差速自转；
3. **脱轨自愈 (Lost-line)**：四路全部为 1 时，小车依次执行：
   $$\text{主动点刹 (0.06s)} \to \text{短距离倒退 (0.10s)} \to \text{沿上次偏航方向原地自转搜索} \to \text{超时 5s 安全急停并退出}$$
4. **异步测量（不停车）**：识别到标记点后，小车分配 `Detection point` 编号，立即在 CSV 中创建 `pending` 行，向后台 C 服务发送采集命令，小车**保持原有路线继续平稳行驶**，采集结果由后台监听线程回调并原子更新。

---

## 三、 CSV 数据输出格式规范

程序每次运行均自动生成带 UTC 时间戳与 UUID 的独立 CSV 文件：
`temperature_points_<RunID>.csv`（`utf-8-sig` 编码）

| 字段名称 | 示例数据 | 含义说明 |
| :--- | :--- | :--- |
| `RunID` | `run_20261009_153022_a1b2` | 本次程序运行全局唯一 ID |
| `Detection point` | `1`, `2`, `3` ... | 从 1 开始严格递增的标记点序号 |
| `Trigger time` | `2026-10-09 15:30:25.123456` | 触发横向标记点的本地微秒时间戳 |
| `Result time` | `2026-10-09 15:30:27.234567` | 传感器测量结果返回的时间戳 |
| `Humidity_%RH` | `55.0` | 实测相对湿度数值 |
| `Temperature_°C` | `23.5` | 实测摄氏温度数值 |
| `Status` | `success` | 事件终态流转（`pending` $\to$ `success` / `sensor_failure` / `timeout` 等） |
| `Explanation` | `Measurement ok` | 状态补充说明或错误详情 |

---

## 四、 启动主控命令
```bash
cd ~/RBE201TC
# 必须以 root 权限运行以获取 GPIO、串口与实时 FIFO 调度权限
sudo python3 line_follower.py
```
按 `Ctrl+C` 触发安全退出机制，程序自动执行双重刹停并安全释放 GPIO 资源。

