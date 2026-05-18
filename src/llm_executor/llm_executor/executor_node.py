# executor.node
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState  # joint_states

# 可选：Jetson 上有，PC 上可能没有 —— 没有则降级为只打印
try:
    from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
except Exception:
    ServosPosition = None
    ServoPosition = None

# 视觉消息（如未安装则降级）
try:
    from vision_interfaces.msg import DetectionResult
except Exception:
    DetectionResult = None

# IK 服务（存在则用于预览求脉冲，不存在也能跑）
try:
    from kinematics_msgs.srv import SetRobotPose as IKSetRobotPose
    _HAS_IK_SRV = True
except Exception:
    IKSetRobotPose = None
    _HAS_IK_SRV = False


def _now_ms() -> int:
    return int(time.time() * 1000)


def _safe_float(x: Any, default: float = 0.0) -> float:
    try:
        if isinstance(x, list) and len(x) == 1:
            x = x[0]
        return float(x)
    except Exception:
        return default


class GroundExecutorNode(Node):
    def __init__(self):
        super().__init__('ground_executor_node')

        # 可重入回调组
        self.cbgroup = ReentrantCallbackGroup()

        # 预览话题 QoS：开启“类似 latched”
        self.qos_preview = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ============== 参数（可在运行时通过 --ros-args -p 覆盖） ==============
        # 运行模式
        self.declare_parameter('dry_run', False)             # 真执行默认 False
        self.declare_parameter('auto_confirm', False)
        self.declare_parameter('require_vision', False)
        self.declare_parameter('vision_timeout_sec', 2.0)

        # 舵机与运动
        self.declare_parameter('servo_topic', '/ros_robot_controller/bus_servo/set_position')
        self.declare_parameter('gripper_id', 10)
        self.declare_parameter('move_duration_ms', 2000)     # 2000ms
        self.declare_parameter('settle_extra_ms', 300)       # 运动完再等 300ms
        self.declare_parameter('pos_eps_rad', 0.02)          # 静止判定阈值（弧度）
        self.declare_parameter('still_window_sec', 0.3)      # 静止窗口

        # 轨迹（抓/放时的相对高度）
        self.declare_parameter('hover_height', 0.08)
        self.declare_parameter('approach_z', 0.015)
        self.declare_parameter('lift_height', 0.08)

        # 话题/节流
        self.declare_parameter('min_goal_interval_ms', 800)
        self.declare_parameter('confirm_topic', '/executor/confirm')
        self.declare_parameter('confirm_str_topic', '/executor/confirm_str')

        # IK 接口参数（与你的手动服务保持一致）
        self.declare_parameter('ik_service', '/kinematics/set_pose_target')
        self.declare_parameter('ik_resolution', 1.0)
        self.declare_parameter('ik_pitch_range', [-180.0, 180.0])
        self.declare_parameter('ik_timeout', 8.0)
        self.declare_parameter('ik_fixed_pitch_deg', 80.0)

        # 确认等待
        self.declare_parameter('confirm_wait_sec', 300.0)

        # 回位脉冲（可按需覆盖）
        self.declare_parameter('home_pulses', [500, 560, 130, 115, 500])

        # ★ ch5（腕）映射参数：直接按 rpy[2]（弧度）映射
        self.declare_parameter('wrist_enable', True)               # 是否启用 ch5 = yaw(rpy)
        self.declare_parameter('wrist_center_pulse', 500)          # 中心脉冲
        self.declare_parameter('wrist_span_deg', 240.0)            # 总行程对应角度（±span/2）
        self.declare_parameter('wrist_limits', [100, 900])         # 脉冲限幅
        self.declare_parameter('wrist_zero_deg', 0.0)              # 机械零位相对世界 0° 的修正（度）
        self.declare_parameter('wrist_clip45', True)               # 是否将角度压到 ±45°（与 sorting 一致）

        # 读取参数
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.auto_confirm = bool(self.get_parameter('auto_confirm').value)
        self.require_vision = bool(self.get_parameter('require_vision').value)
        self.vision_timeout_sec = float(self.get_parameter('vision_timeout_sec').value)

        self.hover_height = float(self.get_parameter('hover_height').value)
        self.approach_z = float(self.get_parameter('approach_z').value)
        self.lift_height = float(self.get_parameter('lift_height').value)

        self.move_duration_ms = int(self.get_parameter('move_duration_ms').value)
        self.settle_extra_ms = int(self.get_parameter('settle_extra_ms').value)
        self.pos_eps_rad = float(self.get_parameter('pos_eps_rad').value)
        self.still_window_sec = float(self.get_parameter('still_window_sec').value)

        self.gripper_id = int(self.get_parameter('gripper_id').value)
        self.servo_topic = str(self.get_parameter('servo_topic').value)

        self.min_goal_interval_ms = int(self.get_parameter('min_goal_interval_ms').value)
        self.confirm_topic = str(self.get_parameter('confirm_topic').value)
        self.confirm_str_topic = str(self.get_parameter('confirm_str_topic').value)

        self.ik_service_name = str(self.get_parameter('ik_service').value)
        self.ik_resolution = float(self.get_parameter('ik_resolution').value)
        self.ik_pitch_range = list(self.get_parameter('ik_pitch_range').value)
        self.ik_timeout = float(self.get_parameter('ik_timeout').value)
        self.ik_fixed_pitch_deg = float(self.get_parameter('ik_fixed_pitch_deg').value)

        self.confirm_wait_sec = float(self.get_parameter('confirm_wait_sec').value)
        self.home_pulses: List[int] = list(self.get_parameter('home_pulses').value)

        # ch5 参数读取
        self.wrist_enable = bool(self.get_parameter('wrist_enable').value)
        self.wrist_center = int(self.get_parameter('wrist_center_pulse').value)
        self.wrist_span_deg = float(self.get_parameter('wrist_span_deg').value)
        self.wrist_limits = list(self.get_parameter('wrist_limits').value)
        self.wrist_zero_deg = float(self.get_parameter('wrist_zero_deg').value)
        self.wrist_clip45 = bool(self.get_parameter('wrist_clip45').value)

        # ============== 发布/订阅 ==============
        # grounded_goal
        self.goal_sub = self.create_subscription(
            String, '/grounded_goal', self.on_goal, 10, callback_group=self.cbgroup)

        # 视觉（可选）
        self.latest_det: Optional[Dict[str, Any]] = None
        if DetectionResult is not None:
            self.vision_sub = self.create_subscription(
                DetectionResult, '/vision_target', self.on_detection, 10, callback_group=self.cbgroup)
        else:
            self.vision_sub = None
            self.get_logger().warn('⚠️ 未找到 vision_interfaces/DetectionResult，视觉确认将不可用')

        # 预览：JSON & 文本 & 逐条（全部使用 TRANSIENT_LOCAL，晚订阅也能收到最近一次）
        self.preview_pub = self.create_publisher(String, '/executor/preview', self.qos_preview)
        self.preview_text_pub = self.create_publisher(String, '/executor/preview_text', self.qos_preview)
        self.preview_step_pub = self.create_publisher(String, '/executor/preview_step', self.qos_preview)
        self.preview_steps_json_pub = self.create_publisher(String, '/executor/preview_steps_json', self.qos_preview)
        self.preview_full_text_pub = self.create_publisher(String, '/executor/preview_full_text', self.qos_preview)

        # 确认
        self.confirm_flag = False
        self.confirm_sub = self.create_subscription(
            Bool, self.confirm_topic, self.on_confirm_bool, 10, callback_group=self.cbgroup)
        self.confirm_str_sub = self.create_subscription(
            String, self.confirm_str_topic, self.on_confirm_str, 10, callback_group=self.cbgroup)

        # 执行完成信号：/executor/done（供 executor_done_sayer 等消费）
        self.declare_parameter('publish_done', True)
        self.publish_done = bool(self.get_parameter('publish_done').value)
        self.done_pub = None
        if self.publish_done:
            self.done_pub = self.create_publisher(Bool, '/executor/done', 10)

        # 舵机发布（实际执行）
        self.servo_pub = None
        if ServosPosition is not None:
            self.servo_pub = self.create_publisher(ServosPosition, self.servo_topic, 10)
        else:
            self.get_logger().warn('⚠️ 未找到 ros_robot_controller_msgs，当前环境将只打印不发布舵机脉冲')

        # IK 客户端（用于预览生成脉冲）
        self._ik_cli = None
        self._ik_srv_cls = IKSetRobotPose if _HAS_IK_SRV else None
        if _HAS_IK_SRV:
            self._ik_cli = self.create_client(IKSetRobotPose, self.ik_service_name, callback_group=self.cbgroup)
            self.get_logger().info(f"🔗 使用 IK 服务: kinematics_msgs/srv/SetRobotPose @ {self.ik_service_name}")
            self._ik_cli.wait_for_service()
        else:
            self.get_logger().warn("⚠️ 未找到 kinematics_msgs/srv/SetRobotPose，预览将使用占位脉冲")

        # 订阅 joint_states：用于“到位”判定
        self.joint_state_sub = self.create_subscription(
            JointState, '/controller_manager/joint_states', self._on_joint_state, 10, callback_group=self.cbgroup)
        self._last_js: Optional[JointState] = None

        # 状态
        self._busy = False
        self._last_goal_hash: Optional[str] = None
        self._last_goal_time_ms: int = 0

        self.get_logger().info(
            "🚀 GroundExecutorNode 启动 "
            f"dry_run={self.dry_run} servo_topic={self.servo_topic} "
            f"auto_confirm={self.auto_confirm} ik_timeout={self.ik_timeout}s "
            f"confirm_wait_sec={self.confirm_wait_sec}s "
            f"(IK 固定: pitch={self.ik_fixed_pitch_deg}°, range={self.ik_pitch_range}, res={self.ik_resolution})"
        )

    # ----------------- ch5：yaw(rpy) → 脉冲（核心改动） -----------------
    def _yaw_to_ch5(self, yaw_rad: float) -> int:
        """将 rpy[2]（弧度）映射到第 5 路舵机脉冲。"""
        if not self.wrist_enable:
            return self.wrist_center
        deg = math.degrees(float(yaw_rad))  # 弧度→度
        # 归一化到 [-180,180)
        deg = (deg + 180.0) % 360.0 - 180.0
        # 与 object_sortting 一致：把角度压到 ±45°，避免 90°跳变
        if self.wrist_clip45:
            if deg < -45.0:
                deg += 90.0
            elif deg > 45.0:
                deg -= 90.0
        # 相对机械零位修正
        deg -= self.wrist_zero_deg
        # 线性映射（±span/2 -> center ± 500）
        k_per_deg = 1000.0 / max(1e-6, self.wrist_span_deg)  # 例如 240° → 1000 脉冲
        pulse = int(round(self.wrist_center + deg * k_per_deg))
        lo = int(self.wrist_limits[0]) if self.wrist_limits else 100
        hi = int(self.wrist_limits[1]) if self.wrist_limits else 900
        pulse = max(lo, min(hi, pulse))
        return pulse

    # 在 IK 的基础上覆盖/补齐 ch5（务必在发布前调用）
    def _finalize_pulses_with_yaw(self, pos: List[float], rpy: List[float], pulses: List[int]) -> List[int]:
        pulses = [int(p) for p in pulses] if pulses else []
        # 确保至少 5 路
        while len(pulses) < 5:
            pulses.append(500)
        # 用 rpy[2] 直接生成 ch5
        ch5 = self._yaw_to_ch5(rpy[2] if (rpy and len(rpy) > 2) else 0.0)
        pulses[4] = ch5
        return pulses

    # ============== JointState 回调与工具 ==============
    def _on_joint_state(self, msg: JointState):
        self._last_js = msg

    def _get_joint_pos(self) -> Optional[List[float]]:
        if self._last_js and self._last_js.position:
            return list(self._last_js.position)
        return None

    def _wait_motion_and_settle(self, duration_ms: int) -> None:
        t_end = time.time() + max(0.0, duration_ms) / 1000.0 + self.settle_extra_ms / 1000.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        start = time.time()
        last = self._get_joint_pos()
        ok_since = None
        while time.time() - start < max(0.05, self.still_window_sec) + 1.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            cur = self._get_joint_pos()
            if last is None or cur is None:
                last = cur
                continue
            diff = max(abs(a - b) for a, b in zip(last, cur)) if last and cur else 1e9
            if diff < self.pos_eps_rad:
                if ok_since is None:
                    ok_since = time.time()
                elif (time.time() - ok_since) >= self.still_window_sec:
                    return
            else:
                ok_since = None
            last = cur

    # ============== 视觉回调 ==============
    def on_detection(self, msg: Any):
        det: Dict[str, Any] = {}
        det['class_name'] = (msg.class_name[0] if getattr(msg, 'class_name', None) else '').lower()
        det['confidence'] = _safe_float(getattr(msg, 'confidence', [0.0]), 0.0)
        det['u'] = float(getattr(msg, 'center_x', [0])[0]) if getattr(msg, 'center_x', None) else 0.0
        det['v'] = float(getattr(msg, 'center_y', [0])[0]) if getattr(msg, 'center_y', None) else 0.0
        z_val = getattr(msg, 'center_z', None)
        det['z'] = _safe_float(z_val, -1.0)
        det['stamp'] = time.time()
        self.latest_det = det
        if det['confidence'] > 0.7:
            self.get_logger().info(f"👁️  视觉: {det}")

    # ============== 确认回调 ==============
    def on_confirm_bool(self, msg: Bool):
        if msg.data:
            self.confirm_flag = True
            self.get_logger().info("✅ 收到确认(Bool)")

    def on_confirm_str(self, msg: String):
        if msg.data.strip().lower() in ('y', 'yes', 'true', 'ok', 'confirm', 'go'):
            self.confirm_flag = True
            self.get_logger().info("✅ 收到确认(String)")

    # ============== grounded_goal 入口 ==============
    def on_goal(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"/grounded_goal JSON 解析失败: {e}")
            return

        status = data.get('status', '')
        if status != 'ok':
            self.get_logger().info(f"[skip] grounded_goal 非成功状态: {status}")
            return

        # 去重/节流
        goal_hash = json.dumps(data, sort_keys=True)
        now = _now_ms()
        if self._last_goal_hash == goal_hash and (now - self._last_goal_time_ms) < self.min_goal_interval_ms:
            self.get_logger().info("[skip] 短时间内重复 grounded_goal，已忽略")
            return
        self._last_goal_hash = goal_hash
        self._last_goal_time_ms = now

        if self._busy:
            self.get_logger().warn("⚠️ 正在执行上一个任务，忽略新的 grounded_goal")
            return

        self._busy = True
        try:
            self._handle_goal(data)
        except Exception as e:
            self.get_logger().error(f"执行异常: {e}")
        finally:
            self._busy = False

    # ============== 主流程 ==============
    def _handle_goal(self, data: Dict[str, Any]):
        # 读源/目标位姿
        src = data.get('source_pose') or {}
        tgt = data.get('target_pose') or {}

        if not src:
            self.get_logger().warn("⚠️ grounded_goal 缺少 source_pose，无法生成轨迹")
            return

        pos_src = list(map(float, src.get('xyz', [0, 0, 0])))
        rpy_src = list(map(float, src.get('rpy', [0, 0, 0])))

        if tgt:
            pos_tgt = list(map(float, tgt.get('xyz', [0, 0, 0])))
            rpy_tgt = list(map(float, tgt.get('rpy', [0, 0, 0])))
        else:
            pos_tgt = None
            rpy_tgt = None

        # 视觉确认（可选）
        if self.require_vision:
            hints = data.get('object_hints', {}) or {}
            if not self._vision_confirm(hints):
                self.get_logger().warn("⚠️ 视觉未确认到目标，跳过此次执行")
                return

        # 关键点序列（打开=200，闭合=700）
        steps: List[Dict[str, Any]] = []
        steps.append(self._mk_move('hover_source', pos_src, rpy_src, dz=self.hover_height))
        steps.append({'type': 'gripper', 'name': 'gripper_open_at_source', 'pulse': 200})
        steps.append(self._mk_move('approach_pick', pos_src, rpy_src, set_z=self.approach_z))
        steps.append({'type': 'gripper', 'name': 'gripper_close_pick', 'pulse': 700})
        steps.append(self._mk_move('lift_after_pick', pos_src, rpy_src, dz=self.lift_height))

        if pos_tgt is not None:
            steps.append(self._mk_move('hover_target', pos_tgt, rpy_tgt, dz=self.hover_height))
            steps.append(self._mk_move('approach_place', pos_tgt, rpy_tgt, set_z=self.approach_z))
            steps.append({'type': 'gripper', 'name': 'gripper_open_place', 'pulse': 200})
            steps.append(self._mk_move('lift_after_place', pos_tgt, rpy_tgt, dz=self.lift_height))

        # 回位（直接按脉冲，不走 IK）
        steps.append({
            'type': 'move',
            'name': 'return_home',
            'pulses': [int(x) for x in self.home_pulses]
        })

        # === 预览：中文描述 ===
        move_dur_ms = int(self.move_duration_ms)
        grip_dur_ms = 300

        def _fmt_xyz(pos: List[float]) -> str:
            return f"({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})"

        def _fmt_pulses(pulses: List[int]) -> str:
            return "(" + "，".join(str(int(p)) for p in pulses) + ")"

        NAME_MAP_MOVE = {
            'hover_source': '源点上方（悬停）',
            'approach_pick': '源点抓取位置',
            'lift_after_pick': '抬起物体',
            'hover_target': '目标上方（悬停）',
            'approach_place': '目标放置位置',
            'lift_after_place': '抬起离开目标',
            'return_home': '初始位'
        }
        NAME_MAP_GRIP = {
            'gripper_open_at_source': ('🟢 开始抓取', '打开夹爪（源点）'),
            'gripper_close_pick': ('🟡 抓取夹紧', '闭合夹爪（抓取）'),
            'gripper_open_place': ('🟣 放置完成', '打开夹爪（放置）'),
        }

        steps_expanded: List[Dict[str, Any]] = []
        preview_lines: List[str] = []

        for st in steps:
            if st.get('type') == 'move':
                if 'pulses' in st and (st.get('position') is None or 'position' not in st):
                    # 回位：仅脉冲
                    pulses = [int(p) for p in st['pulses']]
                    # 强制补齐/覆盖 ch5（用当前阶段的 rpy 或 0）
                    yaw_rpy = (rpy_src if 'source' in st['name'] else (rpy_tgt or [0,0,0]))  # 回家时 rpy 不重要
                    pulses = self._finalize_pulses_with_yaw(st.get('position') or [0,0,0], yaw_rpy, pulses)
                    mode = 'pulses_only'
                    pos = None
                else:
                    pos = st['position']; rpy = st['rpy']
                    self.get_logger().info(f"➡️  IK(preview): {_fmt_xyz(pos)} rpy={rpy}")
                    base = self._ik_get_pulses(pos, rpy)
                    pulses = self._finalize_pulses_with_yaw(pos, rpy, base)  # ★ 这里覆盖/补齐 ch5
                    st['pulses'] = pulses
                    mode = 'pose'

                if st.get('name') == 'return_home':
                    line = f"🏁 回到「{NAME_MAP_MOVE.get('return_home','初始位')}」 · 脉冲={_fmt_pulses(pulses)} · 时长={move_dur_ms/1000:.1f}s"
                else:
                    disp = NAME_MAP_MOVE.get(st.get('name','move'), st.get('name','move'))
                    xyz = _fmt_xyz(pos) if pos is not None else "-"
                    line = f"🔵 移动到「{disp}」 · XYZ={xyz} · 脉冲={_fmt_pulses(pulses)} · 时长={move_dur_ms/1000:.1f}s"

                preview_lines.append(line)

                entry = {
                    'type': 'move',
                    'name': st.get('name', 'move'),
                    'mode': mode,
                    'duration_ms': move_dur_ms,
                    'pulses': [int(p) for p in pulses],
                }
                if mode == 'pose':
                    entry['position'] = [float(pos[0]), float(pos[1]), float(pos[2])]
                    entry['rpy'] = st.get('rpy', [0.0, 0.0, 0.0])
                steps_expanded.append(entry)

            else:
                tag, disp = NAME_MAP_GRIP.get(
                    st.get('name','gripper'),
                    ('🫳 夹爪动作', st.get('name','gripper'))
                )
                pulse = int(st.get('pulse', 200))
                line = f"{tag} · {disp} · ID={self.gripper_id} · 脉冲={pulse} · 时长={grip_dur_ms/1000:.1f}s"
                preview_lines.append(line)

                steps_expanded.append({
                    'type': 'gripper',
                    'name': st.get('name', 'gripper'),
                    'gripper_id': int(self.gripper_id),
                    'pulse': pulse,
                    'duration_ms': grip_dur_ms
                })

        # 发布预览
        full_text = "\n".join(preview_lines)
        self.preview_pub.publish(String(data=json.dumps({'steps': steps_expanded}, ensure_ascii=False)))
        self.preview_text_pub.publish(String(data=full_text))
        self.preview_full_text_pub.publish(String(data=full_text))
        for line in preview_lines:
            self.preview_step_pub.publish(String(data=line))
        self.preview_steps_json_pub.publish(String(data=json.dumps(preview_lines, ensure_ascii=False)))

        self.get_logger().info("📰 已发布预览，等待确认...")
        self.get_logger().info(f"⌛ 等待确认：发布 Bool 到 {self.confirm_topic} (true) 或 String 到 {self.confirm_str_topic} ('yes')")

        if not self._wait_confirm():
            self.get_logger().warn("⛔ 未确认，取消执行")
            return

        # === 实际执行 ===
        for st in steps:
            if st.get('type') == 'move':
                self._exec_move(st)
            else:
                self._exec_gripper(st)

        self.get_logger().info("[done] 执行完成")

        if self.done_pub is not None:
            self.done_pub.publish(Bool(data=True))

    def _mk_move(self, name: str, pos: List[float], rpy: List[float],
                 dz: Optional[float] = None, set_z: Optional[float] = None) -> Dict[str, Any]:
        px, py, pz = pos
        if set_z is not None:
            pz2 = set_z
        elif dz is not None:
            pz2 = pz + dz
        else:
            pz2 = pz
        return {
            'type': 'move',
            'name': name,
            'position': [float(px), float(py), float(pz2)],
            'rpy': [float(rpy[0]), float(rpy[1]), float(rpy[2])]
        }

    # ============== 视觉确认 ==============
    def _vision_confirm(self, hints: Dict[str, Any]) -> bool:
        cls_hint = (hints.get('class') or '').lower()
        deadline = time.time() + self.vision_timeout_sec
        while time.time() < deadline:
            det = self.latest_det
            if det and det.get('confidence', 0.0) >= 0.5:
                cls_det = det.get('class_name', '').lower()
                ok_cls = (not cls_hint) or (cls_hint in cls_det) or (cls_det in cls_hint)
                if ok_cls:
                    self.get_logger().info(f"👁️  视觉确认: hints={hints} det={det}")
                    return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    # ============== IK（预览用） ==============
    def _ik_get_pulses(self, pos: List[float], rpy: List[float]) -> List[int]:
        if self._ik_cli is None or not _HAS_IK_SRV:
            return [500, 500, 500, 500, 500]

        req = IKSetRobotPose.Request()
        if (hasattr(req, 'position') and hasattr(req, 'pitch') and
                hasattr(req, 'pitch_range') and hasattr(req, 'resolution')):
            req.position = [float(pos[0]), float(pos[1]), float(pos[2])]
            req.pitch = float(self.ik_fixed_pitch_deg)
            req.pitch_range = list(self.ik_pitch_range)
            req.resolution = float(self.ik_resolution)
        else:
            return [500, 500, 500, 500, 500]

        future: Future = self._ik_cli.call_async(req)
        deadline = time.time() + max(0.05, self.ik_timeout)
        while rclpy.ok() and time.time() < deadline:
            if future.done():
                res = future.result()
                if res is not None:
                    if hasattr(res, 'pulse'):
                        return [int(p) for p in list(res.pulse)]
                    if hasattr(res, 'pulses'):
                        return [int(p) for p in list(res.pulses)]
                    if hasattr(res, 'positions'):
                        return [int(p) for p in list(res.positions)]
                break
            rclpy.spin_once(self, timeout_sec=0.02)
        return [500, 500, 500, 500, 500]

    # ============== 实际执行（move/gripper） ==============
    def _exec_move(self, st: Dict[str, Any]):
        pulses = st.get('pulses')
        if pulses is not None and ('position' not in st or st.get('position') is None):
            pulses = [int(p) for p in pulses]
            # 回位也确保 ch5 合法（直接保持现有 ch5）
            while len(pulses) < 5:
                pulses.append(500)
            self.get_logger().info(
                f"➡️  MoveTo {st['name']}: pulses={pulses} dur={self.move_duration_ms}ms dry_run={self.dry_run}"
            )
            self._publish_servos(pulses, duration_ms=self.move_duration_ms)
            self._wait_motion_and_settle(self.move_duration_ms)
            return

        # 常规位姿
        pos = st['position']; rpy = st['rpy']
        base = st.get('pulses') or [500, 500, 500, 500, 500]
        pulses = self._finalize_pulses_with_yaw(pos, rpy, base)  # ★ 再次确保 ch5 覆盖
        self.get_logger().info(
            f"➡️  MoveTo {st['name']}: ({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) rpy={rpy} "
            f"dur={self.move_duration_ms}ms dry_run={self.dry_run}"
        )
        self._publish_servos(pulses, duration_ms=self.move_duration_ms)
        self._wait_motion_and_settle(self.move_duration_ms)

    def _exec_gripper(self, st: Dict[str, Any]):
        pulse = int(st.get('pulse', 200))
        name = st.get('name', 'gripper')
        self.get_logger().info(f"🫳 Gripper {name}: id={self.gripper_id} pulse={pulse} dur=300ms dry_run={self.dry_run}")
        if self.dry_run or ServosPosition is None or self.servo_pub is None:
            return
        msg = ServosPosition()
        msg.duration = 0.3
        sp = ServoPosition()
        sp.id = int(self.gripper_id)
        sp.position = int(pulse)
        msg.position = [sp]
        self.get_logger().info(f"📤 Publish Gripper: duration={msg.duration}s items=[({sp.id},{sp.position})]")
        self.servo_pub.publish(msg)
        self._wait_motion_and_settle(int(300))

    # 下发舵机脉冲
    def _publish_servos(self, pulses: List[int], duration_ms: int):
        if self.dry_run or ServosPosition is None or self.servo_pub is None:
            self.get_logger().info(f"📝(Preview) Pulses: {pulses}")
            return
        msg = ServosPosition()
        msg.duration = float(duration_ms) / 1000.0
        arr = []
        for i, p in enumerate(pulses, start=1):
            sp = ServoPosition()
            sp.id = int(i)
            sp.position = int(p)
            arr.append(sp)
        msg.position = arr
        self.get_logger().info(f"📤 Publish Pulses: duration={msg.duration}s items={[ (x.id,x.position) for x in arr ]}")
        self.servo_pub.publish(msg)

    # ============== 确认 ==============
    def _wait_confirm(self) -> bool:
        if self.auto_confirm:
            self.get_logger().info("🤖 auto_confirm=True，自动执行")
            return True
        self.confirm_flag = False
        deadline = time.time() + self.confirm_wait_sec
        while time.time() < deadline:
            if self.confirm_flag:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GroundExecutorNode()
    try:
        exec = MultiThreadedExecutor(num_threads=2)
        exec.add_node(node)
        exec.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

