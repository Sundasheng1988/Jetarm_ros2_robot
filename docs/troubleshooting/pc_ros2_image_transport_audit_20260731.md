# PC 侧 ROS 2 跨机图像接收异常 — 诊断报告 2026-07-31

> 模式：READ-ONLY 诊断。未修改 sysctl / Fast DDS XML / QoS / 网卡 / 路由 / MTU / 防火墙 / 源码 / launch；
> 未重启 PC 或 Orin；未 `ros2 daemon stop`；未清理 `/dev/shm`；未启动 YOLO/ROI/Fusion/Tracker/Runtime；
> 唯一受批准的进程动作：SIGINT 关闭 `rqt_image_view`（PID 15573）。
> 临时探针脚本写入 `/tmp`（`/tmp/ros2_camera_info_probe.py`、`/tmp/ros2_raw_image_probe.py`），**未写入仓库**。
>
> 证据标签：**CONFIRMED** / **UNCONFIRMED** / **UNKNOWN** / **RULED OUT**。
> 不使用 ROOT CAUSE CONFIRMED / FIXED / SAFE / COMPLETE。
> **重要计数口径**：`IpReasmReqds`（收到的**分片**数）与 `IpReasmOKs`（重组成功的**数据报**数）单位不同，
> **禁止相除计算失败率**；重组失败只依据 `IpReasmFails` / `IpReasmTimeout` 及其增量。

---

## Latest Conclusion / Supersession Notice

- 本报告中的 §1–§16 为历史诊断过程，其中部分假设和建议已被后续受控实验推翻。
- 最新有效结论以 §17、§18 为准。
- §12 中"PC 208KB UDP 接收缓冲溢出"为 HIGH 的旧排序已失效。
- §13、§14、§16 中调大系统 rmem 的建议已被 §17 的受控实验判定为无效，不得再次作为下一步动作执行。
- 当前锁定层级：Fast DDS 跨主机 Writer→Reader 大消息传输链路。
- 当前尚未确认的具体机制：Writer 资源/流控、RTPS 分片、XML/profile、locator/interface、history/resource_limits。
- 不得重新执行已完成的 rmem、网络丢包、相机本地速率、PC socket 溢出排查。

---

## 1. Incident Summary

- **现象**：PC（192.168.100.2）能看到 Orin（192.168.100.1）的相机节点与 Publisher（DDS 发现正常），
  但收不到 `/depth_cam/rgb/image_raw` 跨机数据（`ros2 topic hz` 为 0，rqt 画面空）。
- **历史恢复动作**：full PC reboot（用户执行）—— **恢复为暂时性**。
- **Orin 侧**：未重启；故障期间本地相机始终稳定约 30 Hz。
- **当前状态（本会话实测）**：
  - **FAULT ACTIVE AGAIN / DISCOVERY VISIBLE / DATA PLANE (raw) UNAVAILABLE**。
  - 极小消息（camera_info）在无 raw 订阅者时 ~30 Hz 正常；raw 任何订阅下均为 0。
- **根因**：**UNCONFIRMED**（无受控修复前后增量）；但机制有可复现强证据（见第 12 节）。

---

## 2. Confirmed Evidence

| # | 事实 | 标签 | 证据 |
|---|------|------|------|
| E1 | RMW=rmw_fastrtps_cpp，DOMAIN_ID=23，ROS_LOCALHOST_ONLY=0；**无** FASTRTPS/FASTDDS/CYCLONE 环境覆盖 | CONFIRMED | `env` |
| E2 | 无 YOLO/ROI/Fusion/Tracker/runtime 残留进程 | CONFIRMED | `ps -ef \| grep` |
| E3 | PC 可见 `/depth_cam/camera_container`、`/depth_cam/depth_cam` | CONFIRMED | `ros2 node list` |
| E4 | raw/compressed/camera_info Publisher 各 1，均 **RELIABLE / VOLATILE**（Depth UNKNOWN）；GID 前缀 `01.0f`=eProsima | CONFIRMED | `ros2 topic info -v` |
| E5 | eno1 协商 **1 Gbps 全双工**，MTU 1500；去 Orin 路由 `dev eno1 src 192.168.100.2`；neigh REACHABLE | CONFIRMED | `ip`/`/sys/class/net` |
| E6 | wlo1 在 **192.168.3.14/24（家庭 Wi-Fi，不同子网）**；不承载 192.168.100.0/24 | CONFIRMED | `ip -br address` / `ip route` |
| E7 | 无 Fast DDS XML profile / 接口白名单 / `FASTRTPS_DEFAULT_PROFILES_FILE` → 跑默认 | CONFIRMED | `find` + `grep` |
| E8 | Orin eth0=192.168.100.1，**1 Gbps 全双工**，MTU 1500；去 PC 路由 `dev eth0 src 192.168.100.1`；TX 89 MB / 0 err / 0 drop | CONFIRMED | SSH `ip`/`/sys/class/net` |
| E9 | **ufw 状态：不活动；`nft list ruleset` 为空** | CONFIRMED | `sudo ufw status verbose` / `sudo nft list ruleset` |
| E10 | 抓包：来自 192.168.100.1 的 UDP **大量到达 PC eno1**，含 **length 65500 大包 → 192.168.100.2.13161** 及其大量 `ip-proto-17` IP 分片；小包 336B → 13160/13161/13162；多播 239.255.0.1.13150；**0 dropped by kernel** | CONFIRMED | `sudo tcpdump -ni eno1` |
| E11 | Orin 目标地址为 **192.168.100.2（eno1）**，**未**发给 wlo1 的 192.168.3.14 | CONFIRMED | tcpdump |
| E12 | rqt_image_view（PID 15572/15573）持有 13160/13161（raw 图像订阅者）；ros2 daemon（PID 10706）在 13162/13163 | CONFIRMED | `ss -uapn` |
| E13 | `IpReasmFails=0`、`IpReasmTimeout=0`（各次探针增量均为 0）→ **IP 分片重组未失败** | CONFIRMED | `nstat` / `/proc/net/snmp` |
| E14 | `net.core.rmem_max=212992`、`rmem_default=212992`（**208 KB，Ubuntu 默认**）；`netdev_max_backlog=1000` | CONFIRMED | `sysctl` |
| E15 | 停 rqt 后 camera_info：RELIABLE **301 帧**、BEST_EFFORT **303 帧**（均 ~30 Hz）；Publisher RELIABLE→两 QoS 均兼容；RcvbufErrors Δ+25..+35/10s（背景） | CONFIRMED | 探针 + nstat 增量 |
| E16 | raw BEST_EFFORT（唯一订阅者、无 rqt）：**0 帧**；`UdpRcvbufErrors` **Δ+307/12s**；`IpReasmReqds` Δ+592、`IpReasmOKs` Δ+16、`IpReasmFails/Timeout` Δ0；eno1 rx +1.5 MB / err·drop 0 | CONFIRMED | 探针 + nstat/eno1 增量 |
| E17 | raw 探针后 camera_info（同 Python 工具）= 300 帧（~30 Hz），RcvbufErrors Δ+11 → 数据面恢复 | CONFIRMED | 探针 + nstat |
| E18 | softnet 第 2 列（drops）总和=0；内存 31 G/22 G 空闲；load avg≈0.3–1.0；nproc=32 | CONFIRMED | `/proc/net/softnet_stat` / `free` / `uptime` |

---

## 3. Current Process/Node Baseline

- PC 可见节点：`/depth_cam/camera_container`、`/depth_cam/depth_cam`（其余为 Orin 侧硬件节点，跨机可见）。
- PC 侧无 YOLO/ROI/Fusion/Tracker/runtime/grounding 节点运行（E2）。
- `rqt_image_view` 已在本诊断中 SIGINT 关闭（PID 15573 不再存在；13160/13161 已释放）。
- 持续存在：ros2 daemon（PID 10706，端口 13162/13163）—— 仅 CLI 图缓存 Participant，**不订阅图像**。

---

## 4. PC Network and Routing

- 接口：eno1=192.168.100.2/24（UP, 1 Gbps, MTU 1500）；wlo1=192.168.3.14/24（UP，家庭 Wi-Fi）。
- 路由：`192.168.100.0/24 dev eno1 src 192.168.100.2 metric 100`；`default via 192.168.3.1 dev wlo1`。
- `ip route get 192.168.100.1` → `dev eno1 src 192.168.100.2`（正确）。
- `ip rule`：含一条 `220: from all lookup 220`（残留策略，table 220 为空，不影响机器人网段）。
- `ip neigh 192.168.100.1` → `dev eno1 REACHABLE`。
- eno1 计数：RX/TX errors/dropped 均为 0（各次快照）。

---

## 5. Orin Network and Routing

- eth0=192.168.100.1/24（UP, 1 Gbps 全双工, MTU 1500）；wlan0=192.168.149.1/24；l4tbr0=192.168.55.1（DOWN）。
- `ip route get 192.168.100.2` → `dev eth0 src 192.168.100.1`（正确）。
- eth0：RX 28.7 MB / 0 err / 0 drop；**TX 89 MB / 0 err / 0 drop**（Orin 持续大量发送，发送侧无丢包）。
- Orin ROS 环境来自 `.zshrc`（zsh 加载：setup.zsh + ros2_ws + third_party_ros2 + orbbec_ws）；DOMAIN_ID=23。
- Orin 本地 `ros2 topic hz` 未取到（非交互 ssh sourcing 受限，已记录为方法限制，不影响结论；用户已确认 Orin 本地 ~30 Hz）。

---

## 6. Fast DDS / RMW Configuration

- 双侧均 `rmw_fastrtps_cpp`，DOMAIN_ID=23，无 profile/白名单/`FASTRTPS_DEFAULT_PROFILES_FILE`（E1、E7）。
- 即 Fast DDS 使用**默认 UDP 内建传输**：大消息按 ~65500 B 数据报分片，再经 IP 分片（E10）。
- Publisher GID `01.0f...` = eProsima Fast DDS（双侧同厂商、RMW 兼容）。

---

## 7. Topic / QoS Matrix

| Topic | Publisher | Reliability | Durability | PC 接收（无 rqt） | 说明 |
|-------|-----------|-------------|-----------|------------------|------|
| /depth_cam/rgb/camera_info | /depth_cam/depth_cam ×1 | RELIABLE | VOLATILE | RELIABLE 301 / BEST_EFFORT 303（~30 Hz） | 极小消息，正常 |
| /depth_cam/rgb/image_raw | /depth_cam/depth_cam ×1 | RELIABLE | VOLATILE | **BEST_EFFORT 0**（solo） | 大消息，失败 |
| /depth_cam/rgb/image_raw/compressed | /depth_cam/depth_cam ×1 | RELIABLE | VOLATILE | （历史 2.5–4 Hz，本会话 rqt 在线时 0） | 中等消息，边界 |
| /controller_manager/joint_states | /controller_manager | (未取 QoS) | — | rqt 在线时 0 | 小消息，rqt 洪泛时被殃及 |

> Publisher 为 RELIABLE，故 RELIABLE 与 BEST_EFFORT 订阅均兼容；不存在 QoS 不兼容混淆。

---

## 8. UDP / Kernel Counters

- 系统累计（nstat）：`IpReasmFails=0`、`IpReasmTimeout=0`（重组未失败）；`UdpInErrors` 与 `UdpRcvbufErrors` 持续增长且**两者相等**（= 接收 socket 缓冲溢出为主）。
- raw BEST_EFFORT 探针 12 s 增量：`UdpRcvbufErrors` **+307**、`IpReasmReqds` +592、`IpReasmOKs` +16、`IpReasmFails/Timeout` +0。
- camera_info 探针 10 s 增量：`UdpRcvbufErrors` +11..+35（背景级），`IpReasm*` 近乎不变。
- `ipfrag_high_thresh=4194304`、`ipfrag_low_thresh=3145728`、`ipfrag_time=30`、`ipfrag_max_dist=64`（默认）。
- softnet drops=0；eno1 RX err/drop=0。

---

## 9. Multi-NIC Analysis

- wlo1 在 192.168.3.0/24，**与机器人网段不同子网**，无 192.168.100.0/24 路由（E6）。
- tcpdump 证实 Orin 把数据发往 **192.168.100.2（eno1）**，未发往 wlo1 地址（E10、E11）。
- → **多网卡"发现走 eno1、数据走 wlo1"的错 Locator 假设：RULED OUT**。
- 但默认网关在 wlo1（互联网）。Fast DDS 默认仍向正确地址（eno1）发送数据，未受默认路由影响。

---

## 10. Resource / Load Analysis

- 内存 31 G，空闲 22 G，swap 0 使用；load avg 0.28–1.01；nproc=32。→ **PC 资源/CPU/内存非瓶颈**。
- softnet drops=0 → 内核 netdev backlog 未丢包。
- 结论 L（资源耗尽 / 内核网络队列异常）：**RULED OUT**。

---

## 11. Ruled-Out Causes

| 假设 | 状态 | 依据 |
|------|------|------|
| G 防火墙（ufw/nft/iptables） | **RULED OUT** | ufw 不活动、nft 空（E9） |
| B/J 多网卡错误 Locator | **RULED OUT** | Orin 发往 192.168.100.2/eno1（E10、E11） |
| C 路由 / 源地址错误 | **RULED OUT** | `ip route get` 正确，数据到达 eno1 |
| "数据未到达 PC" | **RULED OUT** | tcpdump 见大量 UDP 到达（E10） |
| L PC 资源/内核 backlog | **RULED OUT** | softnet 0、内存/load 正常（E18） |
| E-分片重组失败 | **RULED OUT** | `IpReasmFails/Timeout=0`（E13） |
| H ros2 daemon 为因 | **RULED OUT（弱）** | daemon 不订阅图像；停 rqt 后小消息恢复，daemon 仍在 |
| QoS 不兼容 | **RULED OUT** | Publisher RELIABLE，两种订阅均兼容且 camera_info 双 QoS 均通 |

---

## 12. Ranked Root-Cause Hypotheses

> 机制有可复现证据，但**根因整体仍标 UNCONFIRMED**（未做受控修复前后增量）。

1. **【HIGH，可复现强证据】大数据报 × 208 KB UDP 接收缓冲溢出**
   - 机制：Fast DDS 默认 UDP 传输把 raw Image 切成 ~65500 B 数据报并 IP 分片；PC 端 DDS Participant 的数据 socket 接收缓冲受 `rmem_max=212992`（208 KB 默认）限制，大包洪泛 → `UdpRcvbufErrors` 暴增（raw 探针 +307/12s）→ DDS Reader 收不到完整 Image → 0 帧（E16）。
   - 与 rqt 无关（solo BEST_EFFORT 也 0）、与 RELIABLE 无关（BEST_EFFORT 也失败）、与重组无关（Fails/Timeout=0）。
   - **任何 raw 订阅者（rqt / YOLO / echo）都会触发**；rqt 在线时额外把小消息也拖死（共享路径拥塞），停 rqt 后小消息恢复（E15、E17）。
2. **【MEDIUM】`rmem_max` / `rmem_default` 取 Ubuntu 默认 208 KB**——是假设 1 的具体放大项；对大流量 DDS 偏小。
3. **【LOW，未测】RELIABLE 重传 / ACKNACK 放大**——BEST_EFFORT 已失败，RELIABLE 只会等量或更差（依用户要求未跑 RELIABLE raw）。

---

## 13. Minimum Non-Reboot Recovery Proposal（仅建议，不执行）

不修改配置即可生效的最小动作（按代价升序）：

1. **不在 PC 订阅 raw**：跨机改用 `compressed` / `theora` 或 `image_transport`；raw 仅 Orin 本地消费。
2. **停掉 raw 订阅者**（rqt_image_view / YOLO / echo）：小消息立即恢复（E15、E17 已验证）。
3. （需批准/改配置）临时调大 socket 接收缓冲：
   ```
   sudo sysctl -w net.core.rmem_max=16777216
   sudo sysctl -w net.core.rmem_default=4194304
   ```
   再复测 raw BEST_EFFORT 探针；若恢复 → 证实假设 1（受控前后增量）。

> 以上均为建议；本诊断未执行任何 sysctl / XML / QoS / 网卡变更。

---

## 14. Long-Term Fix Proposal（仅建议）

1. 持久化调大 `net.core.rmem_max` / `rmem_default`（`/etc/sysctl.d/`），并视情况调 `netdev_max_backlog`、`ipfrag_high_thresh`。
2. Fast DDS XML：显式配置 UDP 接收 socket 缓冲；或对大数据启用 **TCP 传输**（避免 IP 分片/UDP 缓冲问题）。
3. 跨机图像策略：默认发布 `compressed`，`raw` 仅本地；或在订阅侧用 `image_transport` 选择压缩传输。
4. 评估切换 RMW（如 Cyclone DDS，分片/缓冲行为不同）作为对照（最后手段）。
5. 复测验证：受控前后取 `UdpRcvbufErrors` / raw hz 增量，确认修复。

---

## 15. Remaining Unknowns

- **UNKNOWN**：未做受控修复前后增量（如调大 rmem 后 raw 是否恢复）→ 根因形式上仍 UNCONFIRMED。
- **UNKNOWN**：raw BEST_EFFORT solo 时 Orin 仅送 ~1.5 MB/12 s（远低于 30 Hz×大图），是否存在 Orin 写端限速 / 匹配 / 租约因素。
- **UNKNOWN**：RELIABLE raw 行为（依用户要求未测；预计等量或更差）。
- **UNKNOWN**：raw 探针期间未实时抓取该 Participant 数据 socket 的 `Recv-Q / skmem`（单线程探针内无法并行 `ss`）。
- **UNKNOWN**：rmem 临界阈值（多少 MB 可消化当前 raw 流）未测。

---

## 16. Recommended Next Command（建议，不执行） — SUPERSEDED BY §17–§18 / DO NOT EXECUTE

受控验证假设 1 的最小命令（**涉及 sysctl，需人工批准；本诊断不执行**）：

```
sudo sysctl -w net.core.rmem_max=16777216 net.core.rmem_default=4194304
# 复测（前后取 UdpRcvbufErrors / raw 增量）
python3 /tmp/ros2_raw_image_probe.py
```

判定：raw 收到帧且 `UdpRcvbufErrors` 增量显著下降 → 证实"大数据报 × 接收缓冲"为因；否则转向 Fast DDS Reader / 传输层。

---

## 状态总结 — SUPERSEDED BY §17–§18 / DO NOT EXECUTE

```text
STATUS:            FAULT ACTIVE AGAIN (raw 跨机接收仍失败；小消息正常)
DISCOVERY:         VISIBLE
DATA PLANE (raw):  UNAVAILABLE（任何 raw 订阅者触发）
RECOVERY ACTION:   full PC reboot（用户执行，历史）→ 恢复为暂时性
ROOT CAUSE:        UNCONFIRMED（无受控修复增量；机制有强可复现证据）
LEADING HYPOTHESIS: 65500B 分片大数据报 × PC 208KB UDP 接收缓冲溢出（HIGH）
```

> 本诊断全程 READ-ONLY（唯一受批准进程动作：SIGINT 关闭 rqt_image_view）。未改任何配置、未重启、未启动 YOLO/Perception/Runtime、未发布任何 Topic。

---

## 17. Phase 4 受控 rmem 前后对照（2026-07-31）

> 本节为**受控干预实验**：临时修改 PC `net.core.rmem_max` / `rmem_default`，复测后**已恢复原值**。
> 本轮仅临时改动 sysctl（已还原）；未改 XML/QoS/MTU/网卡/路由/防火墙；未启动 YOLO/rqt/Perception/Runtime；未发布任何 Topic；全程单一 raw 订阅者（`ss` 仅为只读 socket 观测，非第二订阅者）。
> **结论口径（按本轮指示）：不归入判定 C，记为 SYSTEM-LEVEL RMEM INCREASE INEFFECTIVE / RESULT INCONCLUSIVE / ROOT CAUSE UNCONFIRMED。**
> 计数口径：增量一律以 `/proc/net/snmp` before/after 快照求差为准（Ip 行字段：`ReasmTimeout=$14 ReasmReqds=$15 ReasmOKs=$16 ReasmFails=$17`；Udp 行 `InErrors=$4 RcvbufErrors=$6`）。**不以 `nstat -az` 输出当作测试窗口增量。**

### 17.1 对照条件与恢复

| 阶段 | rmem_max | rmem_default | 说明 |
|---|---|---|---|
| 修改前 | 212992 | 212992 | Ubuntu 默认（208 KB） |
| 修改后 | 16777216 | 4194304 | 系统级 sysctl 已确认生效 |
| 测试后（已恢复）| 212992 | 212992 | 独立只读核验通过；无残留探针；raw `Subscription count: 0` |

### 17.2 raw BEST_EFFORT（唯一订阅者，~12 s 窗口）

| 指标 | 修改前 208 KB | 修改后 16 MB / 4 MB |
|---|---|---|
| received | 0（`no_frame`）| **0（`no_frame`）** |
| `UdpRcvbufErrors Δ` | +47 | +116 |
| `UdpInErrors Δ` | +47 | +116 |
| `IpReasmFails Δ` | 0 | 0 |
| `IpReasmTimeout Δ` | 0 | 0 |
| `IpReasmReqds Δ`（参考，不计失败率）| +367 | +457 |
| `IpReasmOKs Δ`（参考，不计失败率）| +11 | +13 |
| eno1 RX bytes Δ | ≈ 1.13 MB | ≈ 1.36 MB |
| eno1 RX errors/dropped Δ | 0 / 0 | 0 / 0 |

> `UdpRcvbufErrors` 为**系统级全局 UDP 计数，含背景流量**；本轮**不**以单次按字节归一（errors/MB）作为强证据。
> 流量口径：**PC eno1 在该 12 秒窗口收到约 1.36 MB；Orin 实际发送量尚未同步测量。**

### 17.3 小消息对照（camera_info BEST_EFFORT）

- 修改前 received≈301（~30 Hz）；修改后 received≈211（~30 Hz）。
- → 小消息数据面两轮均健康，不受 rmem 变更影响；故障仍锁定在 raw 大数据报路径。

### 17.4 Fast DDS socket 实际缓冲：UNKNOWN

- 本轮 `ss -upnm` 只读观测**未捕获**探针的 DDS 数据 socket（仅见 wlo1 DHCP socket）。
- → Fast DDS 实际数据 socket 的 Recv-Q / skmem / `SO_RCVBUF` 是否采用新系统 rmem：**UNKNOWN（方法学缺口）**。

### 17.5 结论

```text
SYSTEM-LEVEL RMEM INCREASE:  INEFFECTIVE（raw 仍 0 帧；UdpRcvbufErrors 未降）
RESULT:                      INCONCLUSIVE
ROOT CAUSE:                  UNCONFIRMED
```

### 17.6 假设重排（基于本轮受控对照）

| 假设 | 状态 | 依据 |
|---|---|---|
| Ubuntu 系统默认 rmem 太小，单独调大即可修复 | **WEAKENED / NOT SUPPORTED BY CONTROLLED TEST** | rmem 放大 80×（208 KB→16 MB）后 raw 仍 0、`UdpRcvbufErrors` 未降 |
| Fast DDS 实际数据 socket 缓冲或 transport 配置异常 | **UNKNOWN** | 未捕获实际 socket skmem / `SO_RCVBUF` |
| Orin Writer 发送节流 / 匹配 / 同步发布阻塞 | **MEDIUM（待同步发送侧测试）** | 12 s 窗口 PC 仅收到约 1.1–1.36 MB、`IpReasmOKs` 仅 ~11–13，远不足以拼成完整大图帧；Orin 实际发送量未同步测量 |
| IP 重组失败 | **RULED OUT** | 测试窗口 `IpReasmFails Δ=0`、`IpReasmTimeout Δ=0` |

> **对前文的影响（不删改原文）**：第 12 节原领先假设 1「65500B 分片大数据报 × PC 208KB UDP 接收缓冲溢出（HIGH）」已被本轮受控干预**削弱**——单独调大系统 rmem 未恢复 raw。第 12、15 节及「状态总结」的原文与证据**予以保留**，不删除、不覆盖；以本节结论为准。

---

## 18. 同步发送/接收观测 + Orin 本地发布速率（2026-07-31）

> 本节为只读诊断：PC 单一 raw BEST_EFFORT 探针 + Orin 只读 `ip` 计数 / 交互 `ros2 topic hz`。未改 DDS/XML/QoS/sysctl/网卡/源码；未启 YOLO/rqt/ROI/Fusion/Tracker/Runtime；未发布任何 Topic；未留远端订阅进程（Orin `ros2 topic hz` 由远端 `timeout` 自终止，事后 `ps` 核验无残留）。
> **口径收紧**：网卡总计数 ratio≈1.000 **仅能排除明显链路级丢失**，**不得表述为 raw DDS 数据报逐包 100% 收到**。

### 18.1 同步流量观测（两轮 30 s raw BEST_EFFORT 探针）

| 轮次 | Orin eth0 TX Δ | PC eno1 RX Δ | ratio PC_RX/Orin_TX | PC raw received |
|---|---|---|---|---|
| A（含 socket 观测）| ≈2.12 MB | ≈2.12 MB | 1.000 | 1 帧 |
| B（含 hz 尝试）| ≈3.70 MB | ≈3.70 MB | 1.000 | 3 帧 |

- 两轮 Orin 物理发送量均仅 ~2–4 MB/30 s（远低于 29.4 Hz 全量 raw 所需的 ~829 MB/30 s）。
- PC DDS 数据 socket（轮 A：`192.168.100.2:38696`）`Recv-Q=0`、`skmem rb=212992`、drops `d0` → **未溢出**。
- `UdpRcvbufErrors`（轮 A +28）主要来自 **NoMachine（`nxplayer.bin`，单 socket drops d11742）**，非 DDS socket；该计数为系统全局，不作 DDS 缓冲溢出的强证据。

### 18.2 Orin 本地发布速率（交互 `ros2 topic hz`）

- 命令：`ros2 topic hz /depth_cam/rgb/image_raw`（Orin 本机交互，约 31 s）。
- Publisher count=1，Type=`sensor_msgs/msg/Image`。
- **average rate 长期稳定 ~29.2–29.5 Hz**，末值 ~29.45 Hz，window 至 785。
- → **Orin 相机节点与本地 Publisher 工作正常**；本地发布低频假设 **RULED OUT**。
- 备注：首轮经 SSH 非交互运行的 `ros2 topic hz`（远端 timeout 18 s）输出为空，判析为 Jetson 冷启动 `ros2` CLI 占用大部分窗口；改由 Orin 交互运行后取得稳定 29.4 Hz。

### 18.3 最终分层结论

```text
L0 现象        : PC 跨机 raw 仅 1–3 帧/30 s（应为 ~29.4 Hz）
L1 本地发布    : NORMAL（Orin 本地 ~29.4 Hz）→ 相机/发布节点慢 RULED OUT
L2 网络链路    : 无明显链路级丢失（两轮 ratio≈1.000，含口径收紧注记）
L3 PC 接收缓冲 : rmem 放大无效；DDS socket Recv-Q=0/d0 未溢出；RcvbufErrors 主要为 NoMachine 背景 → PC 接收缓冲 RULED OUT（非领先根因）
L4 锁定层      : Fast DDS 跨主机 Writer→Reader 大消息传输链路（发送资源/流控、RTPS 大消息分片、locator/interface 选择、XML/profile）
ROOT CAUSE     : 机制仍 UNCONFIRMED；但根因层级已锁定至 L4
```

### 18.4 排除项汇总（截至本节）

| 假设 | 状态 |
|---|---|
| PC 系统默认 rmem 太小，单独调大即可修复 | **RULED OUT**（受控放大 80× 无效，§17） |
| PC DDS socket 持续溢出 | **RULED OUT**（Recv-Q=0、drops=0） |
| 相机本地发布慢 / 相机节点低频 | **RULED OUT**（本地 ~29.4 Hz） |
| 网络链路明显丢包 | **RULED OUT**（ratio≈1.000，仅排除明显级） |
| IP 分片重组失败 | **RULED OUT**（`IpReasmFails/Timeout` Δ=0） |

### 18.5 下一阶段候选（仅建议，不执行；均需逐项人工批准）

1. Fast DDS **Writer 发送资源 / 流控**（send-socket buffer、asynchronous publish、nack/flow control）。
2. 两端 **Fast DDS XML / profile 与环境变量**（`FASTRTPS_DEFAULT_PROFILES_FILE`、transport、socket buffer、`resource_limits`/`history`）。
3. **多网卡 locator / interface 选择**（白名单、默认路由与 DDS 数据通路是否一致）。
4. **RTPS 大消息分片**与 `resource_limits` / `history` 配置（65500 B 数据报路径）。
5. 必要时做**受控替代 RMW 对照**（如 Cyclone DDS），作为机制判别最后手段。

### 18.6 独立环境问题（单独记录，不与 DDS 根因混淆）

- Orin 执行 `source /opt/ros/humble/setup.bash` 时提示 `/home/ubuntu/ros2_ws/setup.sh: no such file or directory`（setup.bash 内引用了该路径）。
- 本次 `ros2` 命令与 29.4 Hz 测量**仍成功**。
- 此为**独立环境清理项**，不并入当前 DDS 跨机根因结论。

> **以本节（§18）为最新分层结论**：本地 Publisher 正常、跨机 Fast DDS 链路异常；前序各节原文与证据予以保留。
