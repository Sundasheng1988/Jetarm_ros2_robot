# JetArm ROS2 Robot — Topic & Service Map

> 完整 ROS2 通信拓扑图：所有节点的所有 topic 订阅/发布、service 调用/提供、action 服务端

---

## 1. 全局 Topic 通信矩阵

### 1.1 指令/解析/执行链路

```
┌─────────────────────────────────────────────────────────────────────┐
│                         交互入口                                      │
│                                                                      │
│  [keyboard_input_node]                    [speech_dialog_funasr_node]│
│       │                                          │                   │
│  /text_input                              /speech_query              │
│  (String)                                 (String)                   │
│       │                                          │                   │
│       ▼                                          ▼                   │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │              llm_voice_agent (LlmVoiceAgent)                  │  │
│  │                                                                │  │
│  │  Publish:                                                      │  │
│  │    /speech_reply               (String) → TTS                  │  │
│  │    /voice_input/input          (String) → llm_parser           │  │
│  │    /keyboard_input/input       (String) → llm_parser           │  │
│  │    /voice_agent/state          (String)                        │  │
│  │    /tts/interrupt              (Bool)   → TTS                  │  │
│  │    /gesture/cmd                (String) → gesture_player       │  │
│  │    /face_follow/control        (String) → face_follow          │  │
│  │    /servo_controller         (ServosPosition) → controller_mgr │  │
│  └───────────────────────────────────────────────────────────────┘  │
│                                                                      │
│  [llm_command_parser_node]                                           │
│       │ Sub:  /keyboard_input/input, /voice_input/input              │
│       │ Pub:  /parsed_command (String, JSON)                         │
│       ▼                                                              │
│  [grounding_node]                                                    │
│       │ Sub:  /parsed_command                                        │
│       │ Sub:  /world_model/stable_objects                            │
│       │ Sub:  /keyboard_input/input  (raw text fallback)             │
│       │ Pub:  /grounded_goal (String, JSON)                          │
│       │ Pub:  /grounded_task_context (String, JSON)                  │
│       │ Srv:  /grounding/clear_memory (Trigger)                      │
│       ▼                                                              │
│  [ground_executor_node]                                              │
│       │ Sub:  /grounded_goal                                         │
│       │ Sub:  /vision_target (DetectionResult, 可选)                 │
│       │ Sub:  /executor/confirm (Bool)                               │
│       │ Sub:  /executor/confirm_str (String)                         │
│       │ Sub:  /controller_manager/joint_states (JointState)          │
│       │ Pub:  /executor/preview       (String, JSON steps)           │
│       │ Pub:  /executor/preview_text  (String, 多行文本)             │
│       │ Pub:  /executor/preview_step  (String, 逐条步骤)             │
│       │ Pub:  /executor/preview_steps_json (String, JSON array)      │
│       │ Pub:  /executor/preview_full_text (String)                   │
│       │ Pub:  /ros_robot_controller/bus_servo/set_position           │
│       │       (ServosPosition, 直连硬件总线)                          │
│       │ Cli:  /kinematics/set_pose_target (SetRobotPose, IK)         │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 语音全栈内部通信

```
[speech_dialog_funasr_node]
    Sub: /speech_reply        (String, self-speech suppression)
    Sub: /tts_speaking         (Bool, TTS gate)
    Pub: /speech_query         (String, ASR result)
    Pub: /speech_reply         (String, sleep prompt only)
    Pub: /tts/interrupt        (Bool, voice interrupt)

[tts_speaker_node]
    Sub: /speech_reply         (String, text to speak)
    Sub: /tts/interrupt        (Bool, interrupt signal)
    Pub: /tts_speaking         (Bool, gate)
    Pub: /tts/done             (Bool, playback finished)

[executor_done_sayer]
    Sub: /executor/done        (Bool)
    Pub: /speech_reply         (String, "任务已完成")
```

### 1.3 视觉/感知链路

```
[app_compatible_yolo_node]
    Sub: /depth_cam/rgb/image_raw       (Image, 可配置)
    Sub: /depth_cam/depth/camera_info   (CameraInfo, 可配置)
    Cli: /kinematics/get_current_pose   (GetRobotPose)
    Pub: /vision_target                 (DetectionResult)
    Pub: /world_model/objects           (String JSON)

[roi_color_detector_node]   (app package)
    Sub: /depth_cam/rgb/image_raw       (可配置)
    Sub: /depth_cam/rgb/camera_info     (可配置)
    Pub: /roi_color_detector/image_result (Image)
    Pub: /roi_vision_target             (DetectionResult)
    Pub: /world_model/roi_objects       (String JSON)
    Pub: /roi_objects/poses             (PoseArray)
    Pub: /roi_target_pose               (PoseStamped)

[perception_fusion_node]
    Sub: /world_model/objects
    Sub: /world_model/roi_objects
    Pub: /world_model/perception_objects

[stable_object_tracker_node]
    Sub: /world_model/perception_objects
    Pub: /world_model/stable_objects

[object_pose_publisher]     (app package)
    Sub: /depth_cam/rgb/image_raw       (mono8, AprilTag)
    Sub: /depth_cam/rgb/camera_info
    Cli: /kinematics/get_current_pose   (GetRobotPose)
    Pub: /object_pose                   (PoseStamped)
```

### 1.4 世界模型链路

```
[wm_dummy_pub]       Pub: /world_model/objects  (3 个硬编码对象)
[wm_from_tf]          Sub: /tf, /tf_static
                      Pub: /world_model/objects  (TF frame → 对象推断)

[world_model_node]    (social_robot)
    Sub: /env_objects (EnvObjectArray)
    Pub: /world_objects (EnvObjectArray, 经过去重/TTL 融合)

[static_env_report_node] (social_robot)
    Sub: /world_model/objects (String JSON)
    Sub: /voice_input/input (String)
    Pub: /env_objects (EnvObjectArray)
    Pub: /face_follow/control (String)
```

### 1.5 社交/面部链路

```
[face_follow_node]   Sub: /depth_cam/rgb/image_raw (可配置)
                      Sub: /face_follow/control (String: pause/resume/pause_for_action/resume_from_action)
                      Pub: /servo_controller (ServosPosition, 持续)
                      Pub: /face_follow/status (String)

[gesture_player_node] Sub: /gesture/cmd (String: nod/shake)
                       Sub: /face_follow/status (String)
                       Pub: /face_follow/control (String)
                       Pub: /servo_controller (ServosPosition)

[env_scan_node]      Sub: /voice_input/input (String)
                      Sub: /vision_target (DetectionResult)
                      Sub: /world_model/objects (String)
                      Sub: /face_follow/status (String)
                      Pub: /env_objects (EnvObjectArray)
                      Pub: /face_follow/control (String)
                      Pub: /servo_controller (ServosPosition)
```

### 1.6 底层控制链路

```
[controller_manager]
    Sub: /servo_controller        (ServosPosition, pulse/rad/deg)
    Sub: /joint_controller        (JointState, rad)
    Cli: /ros_robot_controller/init_finish (Trigger, wait for HW)
    Pub: ~/joint_states           (JointState, 50Hz)
    Pub: ~/servo_states           (ServoStateList, 50Hz)
    Srv: ~/init_finish            (Trigger)
    Act: arm_controller/follow_joint_trajectory   (FollowJointTrajectory)
    Act: gripper_controller/follow_joint_trajectory

[ServoManager] (内嵌于 controller_manager)
    Pub: ros_robot_controller/bus_servo/set_position (ServosPosition)
    Cli: ros_robot_controller/bus_servo/get_state   (GetBusServoState)

[grasp]  (GraspNode)
    Sub: /grasp                   (Grasp: mode+position+pitch+angle+gripper)
    Cli: /kinematics/set_pose_target (SetRobotPose)
    Pub: /servo_controller        (ServosPosition)
```

### 1.7 其他节点

```
[keyboard_input_node]     Pub: /text_input (String)

[state_to_pulse_node]     Sub: /arm_controller/state (JointTrajectoryControllerState)
                          (仅日志输出，无 topic 发布)

[calibration_node] (app)
    Sub: /depth_cam/rgb/image_raw, /depth_cam/rgb/camera_info
    Cli: /kinematics/get_current_pose (GetRobotPose)
    Srv: ~/enter (Trigger), ~/exit (Trigger), ~/start (Trigger)
    Pub: ~/image_result (Image)
```

---

## 2. 全局 Topic 速查表

| Topic | 类型 | 发布方 | 订阅方 |
|-------|------|--------|--------|
| `/text_input` | `String` | `keyboard_input_node` | *(待下游消费)* |
| `/keyboard_input/input` | `String` | `llm_voice_agent` | `llm_command_parser_node`, `grounding_node`(fallback) |
| `/voice_input/input` | `String` | `llm_voice_agent` | `llm_command_parser_node`, `env_scan_node`, `static_env_report_node` |
| `/parsed_command` | `String` | `llm_command_parser_node` | `grounding_node` |
| `/grounded_goal` | `String` | `grounding_node` | `ground_executor_node` |
| `/world_model/objects` | `String` | `app_compatible_yolo_node`, `wm_dummy_pub`, `wm_from_tf` | `static_env_report_node`, `env_scan_node` |
| `/world_model/roi_objects` | `String JSON` | `roi_color_detector_node` | `perception_fusion_node`; grounding_node (legacy before Sprint 5.6) |
| `/world_model/perception_objects` | `String JSON` | `perception_fusion_node` | `stable_object_tracker_node` |
| `/world_model/stable_objects` | `String JSON` | `stable_object_tracker_node` | `grounding_node` |
| `/grounded_task_context` | `String JSON` | `grounding_node` | `real_grounded_runtime_node` |
| `/runtime/verification_result` | `String JSON` | `verification_result_node` | *(future verification consumer)* |
| `/world_objects` | `EnvObjectArray` | `world_model_node` | *(待消费)* |
| `/env_objects` | `EnvObjectArray` | `env_scan_node`, `static_env_report_node` | `llm_voice_agent`, `world_model_node` |
| `/vision_target` | `DetectionResult` | `app_compatible_yolo_node` | `ground_executor_node`(可选), `env_scan_node` |
| `/roi_vision_target` | `DetectionResult` | `roi_color_detector_node` | *(待消费)* |
| `/roi_objects/poses` | `PoseArray` | `roi_color_detector_node` | *(待消费)* |
| `/roi_target_pose` | `PoseStamped` | `roi_color_detector_node` | *(待消费)* |
| `/roi_color_detector/image_result` | `Image` | `roi_color_detector_node` | *(debug)* |
| `/object_pose` | `PoseStamped` | `object_pose_publisher` | *(待消费)* |
| `/speech_query` | `String` | `speech_dialog_funasr_node` | `llm_voice_agent` |
| `/speech_reply` | `String` | `llm_voice_agent`, `speech_dialog_funasr_node`(sleep), `executor_done_sayer` | `speech_dialog_funasr_node`, `tts_speaker_node` |
| `/tts_speaking` | `Bool` | `tts_speaker_node` | `speech_dialog_funasr_node` |
| `/tts/done` | `Bool` | `tts_speaker_node` | `llm_voice_agent` |
| `/tts/interrupt` | `Bool` | `llm_voice_agent`, `speech_dialog_funasr_node` | `tts_speaker_node` |
| `/executor/preview` | `String` | `ground_executor_node` | *(预览 UI/logger)* |
| `/executor/preview_text` | `String` | `ground_executor_node` | *(预览 UI)* |
| `/executor/preview_step` | `String` | `ground_executor_node` | *(预览 UI)* |
| `/executor/preview_steps_json` | `String` | `ground_executor_node` | *(预览 UI)* |
| `/executor/preview_full_text` | `String` | `ground_executor_node` | *(预览 UI)* |
| `/executor/confirm` | `Bool` | *(用户通过终端/工具发布)* | `ground_executor_node` |
| `/executor/confirm_str` | `String` | *(用户通过终端/工具发布)* | `ground_executor_node` |
| `/executor/done` | `Bool` | *(待 executor 发布)* | `executor_done_sayer` |
| `/servo_controller` | `ServosPosition` | `controller_manager`(转发), `ground_executor_node`, `llm_voice_agent`, `face_follow_node`, `env_scan_node`, `gesture_player_node`, `grasp` | `controller_manager` |
| `/joint_controller` | `JointState` | *(外部控制器)* | `controller_manager` |
| `~/joint_states` | `JointState` | `controller_manager` | `ground_executor_node`(remap), `joint_state_publisher` |
| `~/servo_states` | `ServoStateList` | `controller_manager` | *(debug)* |
| `/ros_robot_controller/bus_servo/set_position` | `ServosPosition` | `ServoManager`, `ground_executor_node` | `ros_robot_controller`(硬件驱动) |
| `/controller_manager/joint_states` | `JointState` | *(controller_manager ~/ remap)* | `ground_executor_node` |
| `/grasp` | `Grasp` | *(外部触发)* | `grasp`(GraspNode) |
| `/gesture/cmd` | `String` | `llm_voice_agent` | `gesture_player_node` |
| `/face_follow/control` | `String` | `llm_voice_agent`, `gesture_player_node`, `env_scan_node`, `static_env_report_node` | `face_follow_node` |
| `/face_follow/status` | `String` | `face_follow_node` | `gesture_player_node`, `env_scan_node` |
| `/depth_cam/rgb/image_raw` | `Image` | 相机驱动 | `app_compatible_yolo_node`, `roi_color_detector_node`, `object_pose_publisher`, `calibration_node`, `face_follow_node` |
| `/depth_cam/rgb/camera_info` | `CameraInfo` | 相机驱动 | `roi_color_detector_node`, `object_pose_publisher` |
| `/depth_cam/depth/camera_info` | `CameraInfo` | 相机驱动 | `app_compatible_yolo_node`, `calibration_node` |
| `/tf` / `/tf_static` | `TFMessage` | TF 广播 | `wm_from_tf` |

---

## 3. 全局 Service 速查表

| Service | 类型 | 服务端 | 客户端 |
|---------|------|--------|--------|
| `/kinematics/set_pose_target` | `SetRobotPose` | `kinematics`(JetArm IK .so) | `ground_executor_node`, `grasp`(GraspNode) |
| `/kinematics/get_current_pose` | `GetRobotPose` | `kinematics`(JetArm) | `app_compatible_yolo_node`, `object_pose_publisher`, `calibration_node` |
| `/grounding/clear_memory` | `Trigger` | `grounding_node` | *(外部/调试)* |
| `/ros_robot_controller/init_finish` | `Trigger` | `ros_robot_controller`(硬件) | `controller_manager` |
| `~/init_finish` | `Trigger` | `controller_manager` | *(等待 controller_mgr 就绪)* |
| `~/enter` / `~/exit` / `~/start` | `Trigger` | `calibration_node` | *(标定工具)* |
| `ros_robot_controller/bus_servo/get_state` | `GetBusServoState` | `ros_robot_controller` | `ServoManager` |

---

## 4. Action 速查表

| Action | 类型 | 服务端 | 客户端 |
|--------|------|--------|--------|
| `arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | `controller_manager` (JointTrajectoryActionController) | MoveIt / 外部 |
| `gripper_controller/follow_joint_trajectory` | `FollowJointTrajectory` | `controller_manager` | MoveIt / 外部 |

---

## 5. 当前 World Model Topic 状态

| 阶段 | Topic | 状态 |
|---|---|---|
| YOLO output | `/world_model/objects` | ✅ semantic source |
| ROI output | `/world_model/roi_objects` | ✅ pose/rpy/color candidate source |
| Fusion output | `/world_model/perception_objects` | ✅ Sprint 5.4 completed |
| Stable tracker output | `/world_model/stable_objects` | ✅ Sprint 5.5 completed |
| Grounding input | `/world_model/stable_objects` | ✅ Sprint 5.6 completed |

**结论**:

Current design (Sprint 5.6 completed):

Camera
→ /depth_cam/rgb/image_raw

ROI
→ /world_model/roi_objects

YOLO
→ /world_model/objects

Fusion
→ /world_model/perception_objects

Stable Tracker
→ /world_model/stable_objects

Grounding
subscribes:
- /world_model/stable_objects
- /parsed_command

publishes:
- /grounded_goal
- /grounded_task_context

Future:

grounded_task_context
↓

real_grounded_runtime

Verification Sidecar (Sprint 6.1 completed):

verification_result_node
subscribes:
- /grounded_task_context
- /world_model/stable_objects
- /executor/done
publishes:
- /runtime/verification_result

VerificationResultNode is observation-only.
It does not control hardware, call IK, or block the runtime.

---

## 6. 数据格式速查

### `SetRobotPose.srv` (kinematics_msgs)
```
request: float64[] position, float64 pitch, float64[] pitch_range, float64 resolution
response: bool success, uint16[] pulse, uint16[] current_pulse, float64[] rpy, float64 min_variation
```

### `GetRobotPose.srv` (kinematics_msgs)
```
request: (empty)
response: bool success, bool solution, geometry_msgs/Pose pose
```

### `DetectionResult.msg` (vision_interfaces)
```
string[] class_name, float32[] confidence, int32[] center_x, int32[] center_y, float32[] center_z
int32[] image_width, int32[] image_height
```

### `Grasp.msg` (servo_controller_msgs)
```
string mode, float64[] position, uint16 pitch, float64 angle, uint16 gripper, uint16 grasp_posture, uint16 pre_grasp_posture
```

### `ServosPosition.msg` (servo_controller_msgs)
```
float64 duration, string position_unit, ServoPosition[] position
```
`position_unit` 可选值: `"pulse"`, `"rad"`, `"deg"`

---

## 7. 控制命令路径总图

```
  [face_follow_node] ─────┐
  [gesture_player_node] ──┤
  [env_scan_node] ────────┤
  [llm_voice_agent] ──────┤
  [grasp] ────────────────┤      ⚠ 无仲裁
  [ground_executor_node] ─┘
       │                    │
       ├─ /servo_controller ───→ [controller_manager] → ServoManager → /ros_robot_controller/bus_servo/set_position
       │
       └─ /ros_robot_controller/bus_servo/set_position ──→ [ros_robot_controller]
           (ground_executor 直连，绕过 controller_manager)
```
