# 🏆 FTC 27570战队 (BAYI RONG ZHEN) - DECODE赛季代码库

![FTC](https://img.shields.io/badge/FIRST_Tech_Challenge-Team_27570-blue.svg)
![Java](https://img.shields.io/badge/Language-Java-orange.svg)
![Status](https://img.shields.io/badge/Status-Archived-red.svg)

欢迎来到北京八一学校 **FTC 27570战队 (BAYI RONG ZHEN)** 的官方开源仓库。

本仓库包含了我们经过高强度比赛检验、深度优化的 FTC 机器人控制软件。我们的代码架构专为极致的稳定性、极高的自动化水平以及前沿的控制理论而设计。

**首席开发者与维护者：**[bluedarkup](https://github.com/bluedarkup)

> **⚠️ 作者 (bluedarkup) 留言：**
> *由于我即将步入高三，需要全力备战高考，非常遗憾我大概率将无法参与下个赛季的 TeamCode 开发工作。编写这套代码是一段令人惊叹的旅程。我选择在这里开源我们的整个框架，希望它能成为 27570 战队未来队员们宝贵的技术遗产，也能为更广泛的 FTC 社区带来灵感。祝大家好运，不断突破极限！*

---

## 🌟 战绩与高光表现
在最近于休斯顿举行的 FTC 世界锦标赛上，27570战队在全球舞台上留下了浓墨重彩的一笔：
*   **FTC 终极巅峰世界锦标赛** 在竞争极其激烈的 **Lovelace 赛区** 中，我们在57支顶尖队伍中排名 **第7**。我们在资格赛中取得了 7胜3负 的骄人战绩，并强势挺进淘汰赛阶段，总共**三次**打破分区最高分记录，历史最高排名世界第四
*   **FTC 中国总决赛：** 在鲁班赛区30支队伍中排名 **第3**，担任联盟队长，展现了极高的自动阶段和手动阶段得分上限。

---

## 🚀 代码库总览：机器人背后的工程学

我们的代码库突破了 FTC SDK 的极限。建立在 **PedroPathing** 轨迹库之上，它拥有高度动态的发射器/云台系统、自定义的 PIDF + 前馈控制环（Feedforward），以及实时的多传感器融合技术。

### 🎯 1. 自动瞄准与运动学预测 (`AutoAimSubsystem` & `AimCalculator`)
这是我们得分机制的大脑，实现了高精度的 **跑打 (Shoot-on-the-Move)** 功能。
*   **目标预测演算：** 算法不是瞄准目标的“当前位置”，而是通过里程计获取机器人自身的速度 (`globalVx`, `globalVy`) 和角速度 (`robotOmega`)，以此来预测发射瞬间的绝对坐标。它将机械发射延迟 (`MECHANICAL_SHOOT_DELAY`) 纳入计算，并动态调整球的飞行时间。
*   **刹车预测：** 对底盘减速进行数学建模 (`FORWARD_BRAKE_DECEL`, `LATERAL_BRAKE_DECEL`)。当操作手松开摇杆时，算法会预判刹车距离，防止云台过度跟枪/甩枪。
*   **多项式插值：** `AimCalculator` 会根据到目标的二维欧几里得距离，动态插值计算出最优的飞轮转速 (RPM) 和俯仰角舵机 (Pitch Servo) 角度。
*   **自定义云台控制器：** 采用多段式控制环。
  *   *远距离阶段：* 标准 PD 控制，用于快速大角度拉枪。
  *   *近距离阶段：* 引入 I-Zone（积分死区）、预测性刹车 (`TURRET_kLinearBraking`, `TURRET_kQuadraticFriction`) 以及自定义前馈控制 (`kS`, `kV`, `kA`)，实现几乎零震荡的死区锁定。

### 🧭 2. 传感器融合定位 (`LimelightPinpointLocalizer`)
*   **纯轮里程计 (Odometry)：** 使用 `GoBildaPinpointDriver` 实现超低延迟、高分辨率的局部位置追踪。
*   **视觉融合 (Limelight 3A)：** 实现了 MegaTag2 全局姿态估计。Limelight 的偏航角 (Yaw) 方向由 Pinpoint IMU 动态实时喂给。操作手可以通过按键（`R3`）将 Pinpoint 的坐标与 Limelight 的绝对坐标进行快照/融合，彻底消除赛局后期的累积漂移。

### 🚗 3. 子系统架构
*   **`FlywheelSubsystem`：** 自定义 PIDF 结合 **Bang-Bang 控制器**，实现爆炸般的起转速度。具有动态 RPM 容差功能（高速时容差更严，低速时容差较宽），确保底层的 `isReady` 标志位在数学上绝对可靠。
*   **`IntakeSubsystem`：** 智能堵转检测。监控电机电流 (Amps)。如果超过 `STALL_CURRENT_AMPS`，会触发短暂的电机停转以保护硬件。它还能与云台同步——当云台需要“反卷”（重置其360度限位）时，进件器会自动反转帮助清理卡弹。
*   **`ClimbSubsystem`：** 搭载 PTO（动力输出）机制。内置 **自动调平算法 (Auto-Leveling Algorithm)**，通过读取 Control Hub IMU 的 Roll（横滚角），对4个爬升电机进行实时差速补偿，使机器人完美平衡悬挂在横杆上。

### 🤖 4. 自动阶段程序 (Autonomous)
我们的自动程序是基于 **PedroPathing** 的非阻塞状态机，能够无缝编排复杂的贝塞尔曲线轨迹。
*   **极致高产路线：**
  *   `BLUEclose21.java` / `REDclose21.java`: 自动程序的皇冠明珠。这是一个令人脑洞大开的 **21球自动路线**，横扫三排元素，完美实现同时进件、跑打发射以及多线程的 RPM 管理。
  *   `BlueClose18Line2.java`: 通过复杂的“开门嘬”动作循环序列，单边稳定输出 18 颗元素。
  *   `BLUE7far.java` / `Custom1Plus5Auto.java`: 极度优化的远端循环路线，智能在角落 A 和角落 B 之间交替，避开拥堵并最大化吸取几何效率。

### 🎮 5. 手动阶段与人机交互 (TeleOp & HMI)
*   **主操作手 (Gamepad 1)：** 全场坐标系麦克纳姆轮驱动，一键 PTO 展开，以及“跑打模式”一键切换。
*   **副操作手 (Gamepad 2)：** 掌控云台微调 (`manualYawOffset`) 和 飞轮转速微调 (`manualRpmOffset`)。完全接管 Limelight 视觉重定位的决策权。
*   **触觉反馈：** 双手柄震动同步，在云台锁定目标以及飞轮达到最佳转速的毫秒级瞬间，直接通过震动通知驾驶员可以开火。
*   **STM32 LED 集成 (`STM32LedInteractiveTest.java`)：** 为菊花链式 STM32 LED 模块（V3 引擎）编写的定制 I2C/串口驱动。将视觉反馈计算从 Control Hub 中剥离，显示进度条、方向流动光效和锁定闪烁，对底层控制主循环（Loop Times）做到**零性能损耗**。

---

## 🛠️ 自动化调参工具 (`/Test`)
我们编写了专门的 OpModes，用数学方法映射机器人的物理极限，彻底告别了繁琐的手动玄学调参：
*   **`TurretFullAutoTuner.java`:** 让云台在不同电压下自动来回摆动并收集数据矩阵。它使用 **最小二乘法回归 (Least Squares Regression)**，自动计算并输出精确的 `kS`（静摩擦力）、`kV`（速度系数）、`kA`（加速度系数）和预测性刹车系数。
*   **`ShooterKSKVTest.java`:** 通过阶梯电压测试，自动映射飞轮的系统阻力曲线。

---

## 📝 部署与安装
1. 将此仓库克隆到本地计算机。
2. 在 Android Studio 中打开项目。
3. 同步 Gradle（请确保您的 FTC SDK 版本与项目匹配）。
4. 使用代码库中确切的硬件映射名称配置您的 Robot Controller（例如：`Turret`, `SH`, `HS`, `Intake`, `odo`, `limelight`, `bbb`, `LP`, `RP`）。
5. **关键步骤：** 在运行任何 Auto 之前，必须先运行 `TurretFullAutoTuner` 和 `ShooterKSKVTest` 获取属于您自己机器的物理常量，并在 `GlobalConstants.java` 中进行更新！

---
### 💙 告别与星辰大海 (Ad Astra)
致 27570 战队未来的队员们：这里的代码属于你们，去拆解它、改进它、重写它。永远不要停止探索 FTC 机器人的极限。