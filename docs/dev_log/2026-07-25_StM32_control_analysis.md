# 2026-07-25 STM32 控制分析（Phase 1 只读审计）

**阶段**：DUAL_TRACK_DIRECTIONAL_VALIDATION → Track A（底盘实物执行异常根因确认）的 STM32 控制链只读审计
**审计日期**：2026-07-26（接入 2026-07-25 方向性隔离调试主线）
**主恢复文件**：`docs/dev_log/2026-07-25_directional_isolation.md`
**本日志配套详细交付物**：`Small_car/analysis/00..04_*.md/.csv`

> 标注：[CF]=Confirmed Fact（直接读得）　[INF]=Inference　[UNK]=Unknown
> 全程**只读**：未修改任何供应商源码、未编译、未执行 BAT/EXE/脚本/二进制、未烧录 HEX、未连串口、未向机器人发命令、未修改 ROS 代码或配置。所有分析输出仅写入 `Small_car/analysis/` 与本 dev_log。

---

## 1. 背景与目标

故障现象：ROS2 发送相同绝对值 `angular.z`，CW（angular.z<0）约 90° 需约 20 s，CCW（angular.z>0）约 90° 需约 48–50 s；同方向重复性好，两方向严重不对称，伴旋转中心偏移与净平移。

ROS 端已确认（不在本次审计范围）：正负 angular.z 同打包路径；linear.x/y、angular.z 各乘 1000 编码为 signed int16；ROS 端无方向分支、单方向比例、方向死区或方向限幅；ROS 端不做四轮速度分解/PWM/PID/编码器闭环。

**本次目标**：对 ROS 串口下发**之后**的 STM32 控制与执行链做 Phase 1 只读审计，定位控制链文件、建立索引、判断可审计性，**不进入根因修改**。

## 2. 审计对象

两个 STM32F407 源码工程（位于 `Small_car/unpacked_unrar/`）：
- **P1** = `stm32_F407VET6_6250_bluetooth/stm32_F407VET6_6250_bluetooth`（4 电机通用固件，M500_4WD 候选）
- **P2** = `T800S_PRO_MC6/T800S_PRO_MC6`（2 电机 T800S_Pro 坦克 + MC6C/SBUS 遥控）

## 3. 工程身份与完整性摘要

- [CF] 两工程各 193 文件，结构完整可读（GBK 注释）。P1 无 `.lib`，控制链全可读；P2 含 `BALANCE/dlrobot.lib`（1,174,244 B，ar 归档，**不可审计**）+ `motor_control.h`。
- [CF] P1 目标 `STM32F407VETx`（12 MHz），与物理板 `STM32F407VET6 12V 控制板 V2.4`（`STM32F407板/STM32F407VET6主板资源分配V2.4.pdf` p.1）一致；P2 目标 `STM32F407ZG`（168 MHz），与 VET6 板封装/容量不匹配。
- [CF] P1 `Robot_Select()` 由**电位器 ADC 运行时**选 6 车型（`robot_select_init.c:14-19`），`M500_4WD` 分支在 `robot_select_init.c:63-64`；4 电机 M1–M4、4 编码器。P2 `Car_Mode=5` 硬编码 Tank_Car（`robot_select_init.c:16`），2 电机 A/B、2 编码器。
- [CF] 两 HEX 不同：P1 `cd87646a…ced1a66`（165,778 B），P2 `0d0ddbfc…0f427d`（152,368 B）。
- [CF] 源码内**无版本字符串/编译日期/固件标识**（grep 无命中）。
- 详见 `Small_car/analysis/00_inventory.md`、`01_project_identity.md`、`03_project_diff.md`。

## 4. 控制链索引摘要

### 4.1 共享 ROS 串口协议
- [CF] 两工程共享同一协议：`FRAME_HEADER 0x7B`/`FRAME_TAIL 0x7D`、`SEND_DATA_SIZE 24`、`RECEIVE_DATA_SIZE 11`。默认 ROS 端口 USART3（P1 `usartx.c:648-709`；P2 `usartx.c:711`，注释 `system.c:143`）。
- [CF] 解包 `XYZ_Target_Speed_transition`（int16×1000→m/s，`/1000`），结果存全局 `Move_X/Y/Z`（P1 `system.c:27`；P2 `system.c:36`）。

### 4.2 P1 控制链（全可读）
`USART3 IRQ → Move_Z → Drive_Motor(4轮分解) → Incremental_PI_A..D → Set_Pwm → TIM1/9/10/11`。
- 运动学 `Drive_Motor`（`balance.c:27-178`）：Mec/FourWheel/Diff/Tank 各支 Vz 系数对称（左右等幅异号）。
- PID 4 路增量 PI（`balance.c:246-249`），增益 `KP=KI=1000`、`Zero_KP=Zero_KI=800`（`system.c:32-33`），限幅 ±15960。
- 编码器 4 路 TIM2/3/4/5，单一比例 `Encoder_precision`，**无正反向差异化**。
- 详见 `02_control_chain_index.md` §A。

### 4.3 P2 控制链（ROS 路径可读；lib 仅 RC 开环）
- [CF] P2 的 **ROS 串口路径全可读**：`USART3/UART4/USART1 → Move_Z → Drive_Motor(2电机差速) → Incremental_PI_A/B → Set_Pwm → TIM9/10/11`。`dlrobot.lib` 对 ROS 路径贡献为 **0**。
- [CF] `dlrobot.lib` 仅暴露 `rc_nonloop_control`/`pwm_rc_nonloop_control`（`motor_control.h:4-5`），仅在 RC 遥控非闭环模式替代运动学+PID（`balance.c:628,574`），ROS 串口路径不调用。
- [CF] P2 **特有方向不对称机制**：正反向编码器差异化比例 `FL/FR/BL/BR_scale`（`balance.c:695-714`、`usartx.c:58-63`）+ `BiasAdjust`（`BiasAdjust.c:72-100`）。P1 可读源码中**未发现**对应机制。
- 详见 `02_control_chain_index.md` §B。

## 5. RECEIVE_DATA 协议核验（证据边界点 6）

- [CF] `RECEIVE_DATA_SIZE=11` 决定 `buffer[11]`；覆盖的 `Control_Str` 声明为 float（Header1+X4+Y4+Z4+Tail1=14B），与 11 表面不一致。
- [CF] **经 union 布局 + 逐字节 IRQ 解析 + 校验和三重核验**：IRQ 把 `buffer[]` 当原始字节，实际帧 `[0]=0x7B 头, [1][2]=保留, [3][4]=X, [5][6]=Y, [7][8]=Z, [9]=XOR(0..8), [10]=0x7D 尾`；`Control_Str` 的 float 字段**从未被读取**。
- [CF] **结论：不是协议错误**。`RECEIVE_DATA_SIZE=11` 为有效帧长，float 结构体为遗留/死声明。与 ROS 端 int16×1000 编码一致。

## 6. 逐项方向不对称审计（证据边界点 3，P1）

| 审计项 | 结论 | 证据 |
|--------|------|------|
| M1..M4_REVERSE | [CF] 0/0/1/1，C/D 取反；常数，非 Z 符号相关 | `robot_select_init.h:209-212`、`balance.c:343-354` |
| M1–M4 → FL/FR/RL/RR 映射 | [UNK] 源码无标注，仅 TIM↔电机注释（TIM2→C、TIM3→D、TIM4→B、TIM5→A） | `system.c:174-185` |
| 每路编码器计数方向 | [CF] HALL1..4_REVERSE=1 全取反 + 每模式取反；常数对称 | `balance.c:870-890` |
| PID 目标/反馈/误差符号 | [CF] 目标=运动学输出，反馈=编码器 m/s，误差 Target−Encoder；符号对称 | `balance.c:246-249,531` |
| ±PID 输出限幅 | [CF] ±15960 对称；stopkey 时 ±3000 对称 | `balance.c:539-540,544-546` |
| PWM 方向位与绝对值 | [CF] 默认分支 16799±value，符号对称，无方向差异化 | `balance.c:397-416` |
| 最小 PWM/死区 | [UNK] P1 可读源码未发现显式最小 PWM/死区（可能由驱动器硬件承担或不存在） | grep 无命中 |
| 刹车/启动补偿 | [CF] stopkey 堵转→±3000、hall_stop→全 0、零速 PI、启动 `Time_count<3000`；均符号对称 | `balance.c:215,268-294,544-546` |
| 四路独立参数一致性 | [CF] 增益四路相同；[INF] 零速分支跨电机引用不一致（A·B、A·D，疑似复制粘贴） | `balance.c:530,535`、`system.c:32-33` |
| **CW/CCW 方向分支** | **[CF] P1 ROS 路径（Mec/Omni/FourWheel/Diff/Tank）无 +Z/−Z 方向分支** | `balance.c:46-177` |

**关键事实（证据边界点 1）**：[CF] P1 源码 ROS 串口路径中**未发现显式 CW/CCW 方向分支**。

## 7. 证据边界（强制约束）

1. [CF] “P1 源码 ROS 路径无显式 CW/CCW 方向分支”为 **Confirmed Fact**。
2. [边界] **不得**据此直接宣布根因一定在 STM32 之外或一定是硬件故障。“无方向分支”是必要非充分条件；下游执行链（PWM/电机/编码器/减速箱/机械阻力/载荷）仍需独立验证。
3. [边界] 仍须逐项审计（见 §6 表），当前已覆盖：M1..M4_REVERSE、映射（部分 UNK）、编码器方向、PID 符号、±限幅、PWM 方向位/绝对值、刹车/启动补偿、四路一致性。**仍待确认**：M1–M4↔物理轮位映射（需原理图/接线）、最小 PWM/死区是否由驱动器硬件承担。
4. [UNK] **当前机器人实际运行固件身份未知**。除非通过 OLED 版本、HEX 哈希比对、供应商确认或读回固件证明，不得假定 P1 或 P2 即为运行固件。
5. [边界] P2 为 T800S_PRO_MC6 双电机履带工程，**其控制逻辑（含正反向差异化比例）不得直接作为 M500 四轮运行事实**。
6. [CF] `RECEIVE_DATA_SIZE=11` 与 float 字段表面尺寸不一致**经核验非协议错误**（见 §5）。

## 8. 第一阶段问题回答

1. **两工程是否完整可读？** [CF] 源码层完整可读；[CF] P2 `dlrobot.lib` 闭源不可审计（但 ROS 路径不依赖它）。
2. **哪个工程更可能对应 M500 底盘？** [INF] **P1**（M500_4WD 分支 + 4 电机 + VET6 与板一致）。P2 为 2 电机 T800S_Pro 坦克。[UNK] 实际运行固件未确认。
3. **是否能在源码中看到 angular.z 到四轮目标速度的完整路径？** [CF] **P1 可以**（`USART3→Move_Z→Drive_Motor→M1..M4.Target`，全可读）；[CF] P2 亦可见 2 电机路径（`Drive_Motor`，可读）。
4. **是否有关键逻辑封装在 dlrobot.lib 中？** [CF] P2 的 `dlrobot.lib` 仅封装 RC 开环 `rc_nonloop_control`/`pwm_rc_nonloop_control`；**ROS 串口路径的运动学/PID/方向/编码器/PWM 全在可读源码**。P1 无 lib。
5. **两个 HEX 是否相同？** [CF] **不同**（§3）。
6. **当前资料是否足以进入第二阶段方向不对称审计？**
   - [INF] 对 **P1**：控制链全可读，Phase 2 可在源码层进行，但需先确认 P1 即运行固件，并补 M1–M4↔轮位映射（原理图/接线）。
   - [INF] 对 **P2**：ROS 路径可读，但其方向不对称机制（正反向比例）属坦克工程，不得套用 M500。
   - [边界] Phase 2 启动前提：**先确认运行固件身份**。
7. **还需向供应商索要哪些资料？** 见 §9。

## 9. 资料缺口 / 需向供应商索取

- [UNK] 机器人实际运行固件版本号/OLED 版本显示约定（源码无版本字符串）。
- [UNK] M1–M4 ↔ 物理轮位（FL/FR/RL/RR）接线表与原理图电机通道对应。
- [UNK] 电机驱动器型号、最小 PWM/死区是否由驱动器硬件实现。
- [UNK] P1 当前车型电位器挡位（运行时 `Car_Mode` 实际选中的是 FourWheel/Mec/… 哪一档）。
- [UNK] `dlrobot.lib` 源码或其内部 RC 开环逻辑说明（若运行 P2 且需审计 RC 路径）。
- [UNK] 编码器线数/减速比/轮径实车确认值（源码为模板宏，随车型分支变化）。

## 10. 下一步（不自动执行，待人工批准）

1. **先确认运行固件身份**（OLED 版本 / HEX 哈希比对 / 供应商确认 / 读回固件），再决定 Phase 2 审计对象是 P1 还是 P2。固件版本确认前**不烧录 STM32**（与 `directional_isolation.md` 禁止事项一致）。
2. Phase 2（P1 方向不对称源码审计）候选深入点：
   - 补 M1–M4↔物理轮位映射（结合 `STM32F407_12V_V2.4原理图.pdf` 与接线）；
   - 核查 P1 运行时 `Car_Mode` 实际挡位（FourWheel_Car vs Tank_Car 运动学差异）；
   - 复核零速分支跨电机引用（`balance.c:530,535`）是否影响四路一致性；
   - 排查最小 PWM/死区是否由驱动器硬件引入方向不对称。
3. **不得**据“P1 无 CW/CCW 分支”宣布根因在 STM32 之外或为硬件故障；Track A 实物执行根因确认仍需技术反馈与硬件级验证。
4. Track B（CW-only/CCW-only 单方向实验）runbook 编写与审核独立进行，待人工批准后先执行 CW-only。

## 11. 文件清单

- `Small_car/analysis/00_inventory.md` — 完整性与工程清单
- `Small_car/analysis/01_project_identity.md` — 工程身份判定
- `Small_car/analysis/02_control_chain_index.md` — 控制链索引（P1+P2，逐项审计）
- `Small_car/analysis/03_project_diff.md` — 两工程差异
- `Small_car/analysis/04_evidence_index.csv` — 证据索引
- `docs/dev_log/2026-07-25_StM32_control_analysis.md` — 本文件（汇总）
