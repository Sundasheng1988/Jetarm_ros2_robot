# ROS2 智能机器人系统 —— src 架构说明

本目录（`src/`）包含整个 ROS2 智能机器人系统的**全部功能包**。  
系统采用 **分层解耦架构**，以保证可扩展性、可维护性与真实机器人系统的一致性。

---

## 🧩 总体分层概览

系统在**逻辑上**分为以下 5 个层级（物理目录不强制隔离，仅在文档层定义）：




[ Interaction / HMI ]
↓
[ Reasoning / Planning ]
↓
[ Application / Behavior ]
↓
[ Perception ]
↓
[ Control / Hardware ]
↓
[ Simulation / Test ]

---

## 1️⃣ 交互层（Interaction / HMI）

**职责**：  
- 接收来自人类的输入（语音、键盘等）
- 将自然输入转化为结构化指令

**特点**：  
- 不直接控制机器人  
- 不关心具体硬件实现  

**包含包**：
- `keyboard_input`：键盘指令输入
- `llm_voice_agent`：语音交互、唤醒词、TTS / ASR
- `llm_parser`：自然语言 → 结构化命令
- `llm_executor`：命令调度与执行决策
- `grounding`：语义到世界对象的绑定（Grounding）

---

## 2️⃣ 执行与硬件控制层（Control / Hardware）

**职责**：  
- 将高层动作转化为底层舵机、控制信号
- 提供稳定、可复用的硬件控制接口

**特点**：  
- 不理解“语义”
- 只执行明确的控制指令

**包含包**：
- `servo_controller`：舵机控制核心
- `joint_pulse_converter`：角度 / 脉冲转换
- `ros_robot_controller_msgs`：机器人控制消息
- `servo_controller_msgs`：舵机消息定义

---

## 3️⃣ 应用层（Application / Behavior）

**职责**：  
- 封装具体机器人能力（抓取、跟随、扫描等）
- 组合多个能力形成完整行为

**特点**：  
- 面向“任务”
- 不直接操作底层舵机

**包含包**：
- `app`：应用级功能节点
- `social_robot`：社交行为（点头、人脸跟随、手势）
- `robot_interfaces`：跨模块通用接口定义

---

## 4️⃣ 感知层（Perception）

**职责**：  
- 感知环境（视觉、目标检测、定位）
- 向上提供结构化感知结果

**特点**：  
- 不做决策
- 不做动作控制

**包含包**：
- `vision_yolo`：YOLO 视觉感知
- `vision_interfaces`：视觉消息接口

---

## 5️⃣ 仿真与测试层（Simulation / Test）

**职责**：  
- 算法验证
- 行为调试
- 非真实硬件环境下的测试

**包含包**：
- `simulations`

---

## 📌 设计原则

- **语义与控制解耦**
- **感知与执行解耦**
- **消息接口优先**
- **真实机器人 / 仿真统一接口**

---

## 📄 说明

- 本 README 仅定义**逻辑架构**
- 实际目录结构保持 ROS2 package 的独立性
- 后续如需物理拆分为 `interaction/`, `perception/` 等目录，可在此基础上平滑演进

