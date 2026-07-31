## Agent 执行策略

使用低上下文开发模式。

### 由用户执行的操作

以下长时间运行或可能影响硬件的命令，由用户执行：

- `ros2 launch`
- 长时间运行的 `ros2 run`
- RViz
- `ros2 bag record`
- 机器人运动、复位和 `/cmd_vel`
- 持续时间超过 30 秒的现场观察

### Agent 可以执行的操作

Agent 可以执行：

- 聚焦的源码检查
- 代码编辑
- 语法检查和单元测试
- 有边界的离线分析
- 指定软件包的构建
- `git status`
- 聚焦的 `git diff`

### 输出和执行要求

- 将冗长的构建、测试和分析输出重定向到日志文件。
- 终端中只汇报：
  - 退出码
  - 关键错误
  - 最多最后 80 行输出
- 不要重复读取已经总结过的文档。
- 不要输出完整源码文件或大段命令输出。
- 每次只处理一个定义明确的任务。
- 编辑文件前，必须先说明：
  - 任务目标
  - 涉及文件
  - 验证方法
- 未经用户明确批准，绝对不能发布 `/cmd_vel` 或操作物理机器人。
- 未经用户明确批准，不得修改 Jetson 上的运行时代码。

# ROS2 机器人项目规则

## 工作区、主机和网络环境

### PC

- Host role: Development / Intelligence / Runtime Host
- Workspace:

  `/home/sundasheng/ros2_ws`

- PC 主要负责：

  - Git 和项目文档
  - LLM / parser / Agent
  - Runtime / Skill / Action
  - Grounding
  - YOLO / ROI
  - Perception Fusion
  - StableObjectTracker
  - Verification Runtime
  - RobotOps
  - RViz
  - rosbag 存储
  - 离线分析工具
  - 单元测试
  - 用户明确指定的软件包构建

### Jetson / Robot Ubuntu

- Host role: Robot-side Hardware Runtime
- Workspace:

  `/home/ubuntu/ros2_ws`

- Jetson 主要负责：

  - `ros_robot_controller`
  - `controller_manager`
  - `servo_manager`
  - `grasp`
  - `kinematics`
  - 深度相机 SDK
  - 移动底盘驱动
  - RPLidar
  - Robot-side TF
  - Robot-side systemd bringup

### ROS 2 Environment

- ROS distribution:

  `ROS 2 Humble`

- ROS Domain:

  `ROS_DOMAIN_ID=23`

- RMW:

  `rmw_fastrtps_cpp`

PC 与 Jetson 必须使用相同的 `ROS_DOMAIN_ID` 和兼容的 RMW 配置。

### Jetson Network Observation — 2026-07-29

Jetson 当前实测地址：

```text
eth0:  192.168.100.1/24
wlan0: 192.168.149.1/24
```

历史临时地址：

```text
172.20.10.2
```

历史地址不得继续作为当前默认 SSH 地址。

每次开发会话开始前，必须重新确认 Jetson 和 PC 的网络地址。

Jetson 侧检查：

```bash
ip -4 addr
ip route
```

PC 侧检查：

```bash
ip -4 addr
ip route
ping -c 3 <当前确认的 Jetson IP>
```

优先使用已经验证的 SSH 别名。

没有验证时，不得根据旧日志推断 Jetson 当前 IP。

当 Ethernet 和 Wi-Fi 同时启用时，必须通过实际 ROS 2 Topic 可见性确认 DDS 通信路径，不得仅根据接口状态推断 DDS 正常。

## ROS 2 Deployment Topology

ROS 2 是分布式系统。

PC 上执行 `ros2 node list` 时看到的是整个 DDS Domain 的综合运行图，不代表这些节点全部运行在 PC。

节点所在主机必须通过对应主机上的进程、systemd、launch 和日志确认。

### Jetson / Robot Ubuntu Nodes

以下节点和功能默认运行在 Jetson：

| Node / Package | Responsibility |
|----------------|----------------|
| `/ros_robot_controller` | STM32 与舵机总线硬件驱动 |
| `/controller_manager` | 接收上层 Servo 命令并管理控制器 |
| `/servo_manager` | 转换并发布底层舵机总线命令 |
| `/grasp` | 原厂 GraspNode |
| `/kinematics` | 原厂 IK 服务 |
| `/buzzer_controller` | Robot-side buzzer |
| Depth Camera SDK nodes | 深度相机驱动和图像发布 |
| `turn_on_dlrobot_robot` | 移动底盘驱动 |
| `mobile_base_bridge` | Odom / TF bridge |
| `rplidar_ros` | LiDAR 驱动 |
| Robot-side TF nodes | 机械臂、相机、底盘和雷达 TF |
| `start_app_node.service` | Robot-side systemd bringup |

Jetson 对以下硬件拥有唯一所有权：

```text
STM32
Servo bus
Mechanical arm
Depth camera device
Mobile base serial device
LiDAR serial device
```

PC 不得启动第二套上述硬件驱动。

### PC Nodes

以下项目节点默认运行在 PC：

| Node / Package | Responsibility |
|----------------|----------------|
| `vision_yolo/simple_yolo_node` | YOLO 语义检测 |
| `roi_color_detector_node` | ROI 颜色、形状与位姿检测 |
| `perception_fusion_node` | YOLO + ROI 融合 |
| `stable_object_tracker_node` | 稳定世界模型 |
| `grounding_node` | 指令到真实目标 Grounding |
| `real_grounded_runtime_node` | Runtime 主执行节点 |
| `verification_result_node` | 任务完成状态验证 |
| `robotops_recorder` | Runtime 事件持久化 |
| LLM / parser / Agent nodes | 自然语言理解和任务输入 |
| RViz | 可视化 |
| Offline analysis tools | rosbag 和数据分析 |

### Deployment Rules

实际运行证据优先于本表。

如果源码、文档和运行状态不一致，必须分别记录：

```text
Expected Host
Observed Host
Process Evidence
Launch Evidence
Unknowns
```

未经用户批准：

- 不得把硬件驱动从 Jetson 迁移到 PC；
- 不得在 PC 启动第二套机械臂、相机、底盘或 LiDAR 驱动；
- 不得把 PC Runtime 自动迁移到 Jetson。

## 移动底盘源码范围

只有以下源码包属于移动底盘：

- `src/turn_on_dlrobot_robot`
- `src/rplidar_ros`
- `src/mobile_base_bridge`

不要把以下目录视为移动底盘代码：

- `src/driver`
- `src/app`
- `src/example`

## 操作规则

### Host-specific Source of Truth

- Jetson 是以下内容的运行事实来源：

  - 机械臂硬件驱动
  - Servo Controller
  - IK
  - Camera SDK
  - Mobile Base
  - LiDAR
  - Robot-side TF
  - Robot-side systemd bringup

- PC 是以下内容的源码和运行事实来源：

  - Runtime
  - Skill / Action
  - Grounding
  - 自定义 Perception
  - Verification
  - RobotOps
  - LLM / parser / Agent
  - 项目文档和测试

- 分布式 ROS 2 系统的整体事实必须同时检查 PC 与 Jetson。

- `ros2 node list` 显示 DDS Domain 的综合运行图，不能单独用于判断节点所在主机。

- 即使 PC 与 Jetson 存在同名文件，也不得假定两边部署版本一致。

- 不要在 Jetson 工作区执行 Git 清理操作。

- 修改机器人代码前，必须先完成：
  - 只读源码调查
  - 运行时状态调查

- 所有结论必须明确区分：

  - Confirmed Fact：已确认事实
  - Direct Observation：直接观察
  - Inference：推断
  - Rejected Hypothesis：已否定假设
  - Unknown：未知

## 有边界的分析模式

当用户明确指定输入文件、rosbag 目录或分析工具时，应将这些路径视为本次任务的白名单。

### 输入范围

- 只分析用户明确指定的输入。
- 只使用用户明确指定的工具，除非用户批准使用其他工具。
- 不得因为工作区中存在相邻目录，就自动检查：
  - 其他源码包
  - 固件
  - 硬件代码
  - 无关的开发日志

### 开发日志使用规则

- 开发日志只用于提供上下文。
- 开发日志不自动构成需要继续执行的任务列表。
- 除非用户明确要求，否则不得恢复开发日志中尚未完成的工作。
- 不得因为读取了某份开发日志，就自动继续其中的旧任务。

### 禁止范围扩展

不得把 rosbag 分析任务扩展为以下内容：

- 源码审计
- STM32 审计
- 电机分析
- 编码器分析
- PID 分析
- PWM 分析
- TF 重构
- SLAM 调试
- AMCL 调试
- Nav2 调试

除非用户明确批准。

### 工具使用规则

- 如果指定工具无法完成某项分析，应直接报告限制。
- 未经用户明确批准，不得为了完成任务而：
  - 设计新算法
  - 编写新分析工具
  - 修改现有工具
  - 搜索替代工具
- 优先执行一次有边界的分析。
- 避免：
  - 重复探索
  - 参数搜索
  - 并行 Agent
  - 后台 Agent
  - 子 Agent
- 当用户提供精确路径时，不得在这些路径之外进行大范围递归搜索。

### 低置信度结果处理

- 分析结果为低置信度或无效时，必须保留原始结果。
- 不得为了得到“看起来有效”的结果而反复调参。
- 不得选择性丢弃不符合预期的结果。
- 无法确认的内容必须标记为 `Unknown`。

### 证据分类

所有分析结果必须明确区分：

- 已确认事实
- 直接观察
- 推断
- 已否定假设
- 未知

## 离线分析工具选择规则

进行任何 rosbag 离线分析前，必须先读取：

`/home/sundasheng/ros2_ws/tools/TOOLS_INDEX.md`

根据该清单选择与当前任务匹配的工具。

- 不得一次运行全部工具。
- 不得根据文件名猜测工具用途。
- 用户明确指定工具时，以用户指定为准。
- 默认只选择一个主工具。
- 只有任务明确同时需要独立LaserScan配准时，才允许增加Scan配准工具。
- 工具不适配时应停止并报告，不得擅自修改工具、搜索替代工具或开发新算法。

在当前任务白名单范围内，Agent可以直接执行：

- `python3 <用户指定的分析脚本> ...`
- `bash <用户指定的辅助脚本> ...`
- 当前任务明确指定工具的子命令，例如：
  - `--help`
  - `inspect`
  - `audit`
  - `register`
  - `analyze`

不得运行来源不明、未检查或不属于当前任务的脚本。

## 任务文件读取与验证权限

### 任务文件读取权限

对于用户当前明确指定的任务，Agent可以读取：

- 用户明确指定的文件和目录
- 当前任务使用的分析工具
- 当前任务指定的开发日志
- 当前任务输入目录中的直接关联文件，例如：
  - `metadata.yaml`
  - `bag_info.txt`
  - `test_conditions.txt`
  - `field_observation.txt`
  - CSV、JSON、Markdown和日志文件
- 当前任务生成的输出文件
- `CLAUDE.md`
- `/home/sundasheng/ros2_ws/tools/TOOLS_INDEX.md`

读取权限仅用于完成当前任务，不代表可以递归检查整个工作区。

如果用户给出精确路径：

- 该路径视为允许读取的白名单。
- 可以读取该目录下与任务直接相关的文件。
- 不得自动扩展到相邻项目、其他测试目录或无关源码。

### 允许的低风险Shell命令

Agent可以在当前任务允许的路径内使用以下只读或低风险命令：

- `pwd`
- `ls`
- `tree`
- `find`
- `stat`
- `file`
- `du`
- `wc`
- `cat`
- `head`
- `tail`
- `less`
- `grep`
- `rg`
- `sed -n`
- `awk`
- `cut`
- `sort`
- `uniq`
- `column`
- `echo`
- `printf`
- `realpath`
- `readlink`
- `sha256sum`
- `md5sum`
- `diff`
- `cmp`
- `jq`
- `python3 -m json.tool`
- `ros2 bag info`
- `ps`
- `pgrep`
- `pstree`

使用要求：

- `find`必须限制在任务白名单路径内。
- 大目录优先使用 `-maxdepth`。
- `grep`或`rg`不得无边界扫描整个工作区。
- 不得完整打印大型源码、CSV、JSON或日志。
- 大型输出必须重定向到日志文件，只汇报摘要和最多最后80行。
- 可以将不需要显示的输出重定向到 `/dev/null`。


读取文件或目录前应先评估规模：

- 大文件优先使用 `wc -l`、`stat`、`head`、`tail` 或 `sed -n`。
- 超过 300 行的文件不得直接完整使用 `cat` 输出。
- `tree` 和 `find` 默认必须限制深度和路径范围。
- 单次终端输出不得超过 80 行。
- 更长内容必须重定向到日志文件，只汇报摘要、关键错误和最多最后 80 行。

对于上述明确允许的只读、低风险和离线验证操作，Agent可以直接执行，
无需逐条向用户请求确认。

但如果命令：

- 超出当前任务白名单路径；
- 可能覆盖已有文件；
- 预计运行超过5分钟；
- 可能影响机器人、Jetson运行时或Git历史；

则必须先获得用户批准。

### 允许创建的文件和目录

在用户指定的输出目录内，Agent可以：

- 使用 `mkdir -p` 创建分析输出目录
- 使用 `touch` 创建空文件
- 使用 `echo`、`printf`、重定向或 `tee` 写入：
  - 日志
  - Markdown报告
  - CSV
  - JSON
  - 临时配置
  - 测试结果
- 创建任务所需的临时目录
- 创建语法检查和单元测试产生的缓存文件

所有新文件必须写入：

- 用户明确指定的输出目录；或
- 当前任务明确批准的临时目录。

不得把临时文件散落到无关源码目录。

### 代码编辑权限

当用户明确要求修改代码时，Agent可以：

- 读取指定源码文件
- 编辑指定源码文件
- 新建用户明确要求的新文件
- 运行聚焦的格式、语法和单元测试
- 查看相关 `git diff`

编辑前必须先说明：

1. 修改目标
2. 涉及文件
3. 计划修改内容
4. 验证方法

未经用户批准，不得顺带修改其他文件。

### 允许的验证操作

Agent可以执行以下有边界的离线验证：

- `python3 -m py_compile <指定文件>`
- 对指定脚本运行 `--help`
- 对指定模块运行聚焦的单元测试
- 对指定测试文件运行 `pytest`
- 对指定ROS2软件包执行构建
- 对指定rosbag运行离线分析
- 读取分析结果并进行数值一致性检查
- 运行 `git status`
- 运行聚焦的 `git diff -- <指定文件>`

验证必须满足：

- 不启动机器人
- 不发布ROS消息
- 不播放bag到实时ROS网络，除非用户明确批准
- 不运行持续时间不可控的任务
- 不并行启动多个重计算任务
- 预计超过2分钟的命令，执行前说明预计耗时
- 预计超过5分钟的命令，必须先获得用户批准
- 分析命令一次只运行一个

### ROS2软件包构建输出规则

执行用户明确批准的软件包级 ROS2 构建时，允许构建系统写入 PC 工作区的标准目录：

- `/home/sundasheng/ros2_ws/build`
- `/home/sundasheng/ros2_ws/install`
- `/home/sundasheng/ros2_ws/log`

构建规则：

- 只允许构建用户明确指定的软件包。
- 优先使用：

  `colcon build --packages-select <package_name>`

- 不得默认构建整个工作区。
- 不得因为构建一个软件包而修改其他软件包源码。
- 构建输出必须重定向到日志文件。
- 终端只汇报退出码、关键错误和最多最后 80 行。
- 不得在 Jetson 工作区执行构建，除非用户明确批准。
- 不得修改或清理现有 `build/`、`install/`、`log/` 内容，除非用户明确批准。

### 临时文件与清理

Agent可以在任务输出目录中创建：

- `tmp/`
- `logs/`
- `artifacts/`

但不得自动删除以下内容：

- 原始rosbag
- 用户文件
- 已确认分析结果
- 开发日志
- 源码
- Git跟踪文件

以下操作必须由用户明确批准：

- `rm`
- `rm -rf`
- 覆盖已有重要文件
- 移动或重命名原始数据
- 清理Git状态
- 删除旧工具
- 删除分析结果

### systemd 只读调查权限

Agent 可以在 Jetson 上直接执行以下只读命令：

```bash
systemctl status <service>
systemctl cat <service>
systemctl show <service>
journalctl -u <service> --no-pager
```

使用要求：

- 日志必须限制在当前指定服务；
- 使用 `--since`、`-n`、`head` 或 `tail` 限制输出；
- 单次终端展示不得超过 80 行；
- 只允许读取状态，不得改变服务状态。

未经用户明确批准，不得执行：

```text
systemctl start
systemctl stop
systemctl restart
systemctl enable
systemctl disable
systemctl mask
systemctl unmask
```

### Hardware Device Inspection

Agent 可以执行以下只读设备枚举：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
readlink -f /dev/serial/by-id/* 2>/dev/null
```

这些命令只允许查看：

- 设备是否存在；
- 符号链接目标；
- 设备权限；
- 稳定设备名称。

未经用户明确批准，Agent 不得：

- 打开串口；
- 向设备写数据；
- 使用 `echo`、`cat`、Python serial 或其他程序读取或写入设备内容；
- 修改设备权限；
- 创建或修改 udev 规则；
- 操作 GPIO、CAN、PWM、I2C、SPI 或 Servo 设备。

### 明确禁止的命令和操作

未经用户明确批准，不得执行：

- `sudo`
- `apt`
- `apt-get`
- `pip install`
- `conda install`
- `npm install`
- `curl`
- `wget`
- 网络上传或下载
- `chmod`或`chown`
- `dd`
- `mkfs`
- `mount`
- `umount`
- `reboot`
- `shutdown`
- `kill`或`pkill`
- `git reset`
- `git clean`
- `git checkout --`
- `git restore`
- `git rebase`
- `git push`
- 修改Jetson运行时代码
- 写入或访问硬件设备节点

- 修改 systemd 状态的命令，包括：
  - `systemctl start`
  - `systemctl stop`
  - `systemctl restart`
  - `systemctl enable`
  - `systemctl disable`
  - `systemctl mask`
  - `systemctl unmask`

## Jetson Agent Mode

Agent 在 `/home/ubuntu/ros2_ws` 中默认处于 `READ-ONLY` 模式。

### 允许

- 查看指定源码；
- 查看 launch 和配置；
- 查看 systemd 只读状态；
- 查看进程树；
- 查看 ROS 2 graph；
- 查看串口设备名称和符号链接；
- 输出审查报告到用户指定目录。

### 未经用户明确批准禁止

- 编辑 Jetson 源码；
- 构建 Jetson 工作区；
- 修改 `build/`、`install/` 或 `log/`；
- 启动、停止或重启 systemd 服务；
- 启动或停止 ROS 2 launch；
- 发布机械臂控制消息；
- 发布 `/cmd_vel`；
- 修改串口、udev 或网络配置；
- 执行 Git 清理、恢复或历史修改；
- 访问或写入硬件设备内容。

即使 PC 中存在同名源码，也不得假定 PC 与 Jetson 的部署版本一致。

当前重复控制栈调查只允许：

```text
源码读取
launch 树读取
systemd 只读调查
进程树调查
ROS 2 graph 调查
报告输出
```

### 任务完成规则

完成当前任务后，Agent必须：

1. 汇报读取了哪些输入
2. 汇报创建或修改了哪些文件
3. 汇报执行了哪些验证
4. 给出退出码和关键结果
5. 标记未完成或未知的部分
6. 停止执行，不自动开始下一项任务