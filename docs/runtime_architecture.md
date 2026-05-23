# JetArm Robot Runtime — Architecture Diagrams

> 基于 runtime_analysis.md、topic_service_map.md、refactor_plan.md 的图形化表达
> 使用 Mermaid graph TD 语法，全部为 Markdown 内嵌文本图

---

## 1. 系统总体架构图

```mermaid
graph TD
    subgraph PC["🖥️ PC Ubuntu — 上层逻辑 / 感知 / 交互"]
        subgraph INTERACTION["交互层"]
            VOICE["llm_voice_agent<br/>语音全栈<br/>ASR(FunASR)+LLM(Ollama)+TTS(Piper)"]
            KB["keyboard_input_node<br/>终端键盘输入"]
        end

        subgraph REASONING["推理层"]
            PARSER["llm_command_parser_node<br/>关键词匹配解析<br/>/parsed_command"]
            VA_LLM["llm_voice_agent<br/>槽位提取+confirm<br/>/keyboard_input/input<br/>/voice_input/input"]
        end

        subgraph GROUNDING_LAYER["Grounding 层"]
            GROUNDING["grounding_node<br/>对象匹配+目标位姿<br/>/grounded_goal"]
            WM_D["wm_dummy_pub<br/>硬编码仿真对象<br/>/world_model/objects"]
            WM_TF["wm_from_tf<br/>TF→对象推断<br/>/world_model/objects"]
        end

        subgraph PERCEPTION["感知层"]
            YOLO["app_compatible_yolo_node<br/>YOLOv8+LAB方块检测<br/>/vision_target<br/>/world_model/objects"]
            ROI["roi_color_detector_node<br/>ROI颜色检测<br/>/world_model/roi_objects<br/>/roi_vision_target"]
        end

        subgraph EXECUTION["执行层 — 核心写死区"]
            EXECUTOR["ground_executor_node<br/>硬编码 pick-and-place<br/>IK调用+舵机脉冲<br/>/executor/preview*"]
        end

        subgraph SOCIAL["社交层"]
            FACE["face_follow_node<br/>MediaPipe人脸跟随"]
            GESTURE["gesture_player_node<br/>nod/shake手势"]
            ENV["env_scan_node<br/>环境扫描"]
        end
    end

    subgraph JETARM["🦾 JetArm Orin Nano — 底层控制 / 硬件驱动"]
        subgraph CTRL["控制层"]
            CM["controller_manager<br/>关节控制+轨迹跟随<br/>~/joint_states"]
            SM["ServoManager<br/>舵机脉冲管理<br/>rad↔pulse转换"]
            GRASP_NODE["grasp (GraspNode)<br/>夹爪序列控制<br/>/servo_controller"]
        end

        subgraph IK["运动学"]
            IK_SRV["kinematics IK .so 服务<br/>/kinematics/set_pose_target<br/>/kinematics/get_current_pose"]
        end

        subgraph HW["硬件驱动"]
            RRC["ros_robot_controller<br/>硬件总线驱动<br/>/bus_servo/set_position"]
        end

        subgraph CAMERA["相机"]
            DEPTH_CAM["深度相机驱动<br/>/depth_cam/rgb/image_raw<br/>/depth_cam/depth/camera_info"]
        end

        STM32["STM32 固件"]
    end

    %% 指令链路
    KB -->|"/text_input"| VOICE
    VOICE -->|"/keyboard_input/input"| PARSER
    VOICE -->|"/voice_input/input"| PARSER
    VA_LLM -->|"/keyboard_input/input<br/>/voice_input/input"| PARSER
    PARSER -->|"/parsed_command<br/>String JSON"| GROUNDING

    %% Grounding 链路
    WM_D -.->|"/world_model/objects<br/>⚠ 不匹配"| GROUNDING
    WM_TF -.->|"/world_model/objects<br/>⚠ 不匹配"| GROUNDING
    YOLO -.->|"/world_model/objects<br/>⚠ 不匹配"| GROUNDING
    ROI -->|"/world_model/roi_objects<br/>✅ 匹配"| GROUNDING
    GROUNDING -->|"/grounded_goal<br/>String JSON"| EXECUTOR

    %% 视觉链路
    DEPTH_CAM -->|"Image"| YOLO
    DEPTH_CAM -->|"Image"| ROI
    YOLO -->|"/vision_target<br/>DetectionResult"| EXECUTOR

    %% 社交链路
    DEPTH_CAM -->|"Image"| FACE
    VOICE -->|"/gesture/cmd"| GESTURE
    VOICE -->|"/face_follow/control"| FACE
    VOICE -->|"/voice_input/input"| ENV

    %% 执行链路
    EXECUTOR -->|"/ros_robot_controller/bus_servo/set_position<br/>⚠ 绕过 controller_manager"| RRC
    EXECUTOR -->|"/servo_controller"| CM
    CM -->|"~/joint_states"| EXECUTOR

    %% IK 链路
    EXECUTOR -->|"Client: /kinematics/set_pose_target"| IK_SRV
    GRASP_NODE -->|"Client: /kinematics/set_pose_target"| IK_SRV
    YOLO -->|"Client: /kinematics/get_current_pose"| IK_SRV
    FACE -->|"/servo_controller"| CM
    GESTURE -->|"/servo_controller"| CM
    ENV -->|"/servo_controller"| CM
    GRASP_NODE -->|"/servo_controller"| CM

    %% 底层硬件链路
    CM --> SM
    SM -->|"ServosPosition"| RRC
    RRC --> STM32

    %% 标注 topic 不匹配
    style GROUNDING fill:#fff3cd,stroke:#ffc107
    style EXECUTOR fill:#f8d7da,stroke:#dc3545
    style CM fill:#d1ecf1,stroke:#0c5460
    style SM fill:#d1ecf1,stroke:#0c5460
    style IK_SRV fill:#d1ecf1,stroke:#0c5460
    style RRC fill:#d1ecf1,stroke:#0c5460
    style STM32 fill:#d1ecf1,stroke:#0c5460
```

---

## 2. 用户指令执行链路图

```mermaid
graph TD
    subgraph INPUT["输入入口"]
        VOICE_MIC["🎤 麦克风<br/>PCM 16kHz Mono"]
        KEYBOARD_IN["⌨️ 键盘终端"]
    end

    subgraph ASR["语音识别 llm_voice_agent"]
        VAD["WebRTC VAD<br/>aggressiveness=3"]
        FUNASR["FunASR<br/>paraformer-large<br/>+ fsmn_vad + punc"]
        WAKE["Wake Word 检测<br/>rebecca / 瑞贝卡"]
        FILTER["后处理<br/>self-speech suppress<br/>confidence≥0.75<br/>≥4 chars"]
        ASR_OUT["/speech_query"]
    end

    subgraph VA_AGENT["llm_voice_agent Node"]
        LAYERS["L1 pause/resume<br/>L2 wake/sleep<br/>L3 mute/unmute"]
        MODE["mode: chat | task"]
        SLOT_EXTRACT["槽位提取<br/>COLOR_MAP+CLASS_MAP+SIDE_MAP"]
        CONFIRM["确认对话<br/>confirm/cancel"]
        VA_PUB["/voice_input/input<br/>/keyboard_input/input"]
    end

    subgraph PARSER_STAGE["指令解析 llm_parser"]
        NORM["re.sub 去中文间空格<br/>(Vosk分词残留)"]
        PARSE_FN["parse() 关键词匹配<br/>颜色→COLOR_MAP<br/>类别→CLASS_MAP<br/>目标→SIDE_MAP"]
        CMD_ASSEMBLE["组装<br/>action='pick' (固定)<br/>from='red_cube'<br/>to='right_side'"]
        PARSE_OUT["/parsed_command<br/>String JSON"]
    end

    subgraph GROUNDING_STAGE["Grounding grounding_node"]
        SPLIT_FROM["split_from_token(from)<br/>→ class, color"]
        SEL_OBJ["_select_object()<br/>模糊子串匹配<br/>按confidence↓/recency↓/proximity↑排序"]
        PLACE_MAP["place_map[to_side]<br/>→ target_pose<br/>frame+xyz+rpy"]
        VALIDATE["规则校验<br/>pick: need obj+target<br/>place: need obj<br/>move: need target"]
        GOAL_OUT["/grounded_goal<br/>String JSON<br/>{intent,object_id,source_pose,target_pose,status}"]
    end

    subgraph EXECUTOR_STAGE["执行器 ground_executor_node"]
        VALIDATE_GOAL["status='ok'?<br/>去重 800ms<br/>busy check"]
        VISION_CONF["可选视觉确认<br/>_vision_confirm()<br/>/vision_target"]
        BUILD_STEPS["硬编码10步序列"]
        IK_CALL["对每步move调IK<br/>/kinematics/set_pose_target<br/>position+pitch+range+resolution"]
        CH5_MAP["_yaw_to_ch5()<br/>rpy[2]→腕部脉冲<br/>clip±45°"]
        PREVIEW["发布预览×5 topic<br/>/executor/preview*"]
        WAIT_CONF["等待确认<br/>/executor/confirm Bool<br/>/executor/confirm_str String"]
        EXEC_STEPS["_exec_move() + _exec_gripper()"]
        MONITOR["_wait_motion_and_settle()<br/>读 /controller_manager/joint_states"]
    end

    subgraph IK_STAGE["逆运动学 kinematics"]
        IK_SOLVE["IK .so 求解<br/>SetRobotPose.srv<br/>position pitch pitch_range resolution"]
        PULSE_OUT["pulse[]<br/>[p1,p2,p3,p4,p5]"]
    end

    subgraph SERVO_STAGE["舵机控制层 servo_controller"]
        CM_PROC["controller_manager<br/>处理 ServosPosition<br/>pulse|rad|deg→pulse"]
        SM_PROC["ServoManager<br/>钳位 duration [0.02,30]s<br/>钳位 position [0,1000]"]
        PUB_CMD["/ros_robot_controller/bus_servo/set_position<br/>ServosPosition"]
    end

    subgraph HW_STAGE["硬件执行"]
        RRC_DRV["ros_robot_controller<br/>硬件驱动"]
        STM32_FW["STM32 固件"]
        SERVOS["舵机 1-5 臂<br/>舵机 10 夹爪"]
    end

    %% 连线
    VOICE_MIC --> VAD
    VAD --> FUNASR
    FUNASR --> WAKE
    WAKE --> FILTER
    FILTER --> ASR_OUT
    ASR_OUT --> LAYERS
    KEYBOARD_IN --> NORM

    LAYERS --> MODE
    MODE -->|"task mode"| SLOT_EXTRACT
    SLOT_EXTRACT --> CONFIRM
    CONFIRM --> VA_PUB

    VA_PUB -->|"/keyboard_input/input"| NORM
    NORM --> PARSE_FN
    PARSE_FN --> CMD_ASSEMBLE
    CMD_ASSEMBLE --> PARSE_OUT

    PARSE_OUT --> SPLIT_FROM
    SPLIT_FROM --> SEL_OBJ
    SEL_OBJ --> PLACE_MAP
    PLACE_MAP --> VALIDATE
    VALIDATE --> GOAL_OUT

    GOAL_OUT --> VALIDATE_GOAL
    VALIDATE_GOAL --> VISION_CONF
    VISION_CONF --> BUILD_STEPS
    BUILD_STEPS --> IK_CALL
    IK_CALL --> PULSE_OUT
    PULSE_OUT --> CH5_MAP
    CH5_MAP --> PREVIEW
    PREVIEW --> WAIT_CONF
    WAIT_CONF --> EXEC_STEPS
    EXEC_STEPS --> MONITOR

    PULSE_OUT -.-> CM_PROC
    EXEC_STEPS -->|"/ros_robot_controller/bus_servo/set_position<br/>⚠ 直连硬件总线"| SM_PROC
    CM_PROC --> SM_PROC
    SM_PROC --> PUB_CMD
    PUB_CMD --> RRC_DRV
    RRC_DRV --> STM32_FW
    STM32_FW --> SERVOS

    style IK_SOLVE fill:#d1ecf1,stroke:#0c5460
    style CM_PROC fill:#d1ecf1,stroke:#0c5460
    style SM_PROC fill:#d1ecf1,stroke:#0c5460
    style RRC_DRV fill:#d1ecf1,stroke:#0c5460
    style STM32_FW fill:#d1ecf1,stroke:#0c5460
    style SERVOS fill:#d1ecf1,stroke:#0c5460
    style BUILD_STEPS fill:#f8d7da,stroke:#dc3545
    style EXEC_STEPS fill:#f8d7da,stroke:#dc3545
```

---

## 3. 当前系统 Topic / Service 流图

```mermaid
graph TD
    subgraph TOPICS["📡 Topic 发布/订阅关系"]
        subgraph TOPIC_CMD["指令链路 Topics"]
            T_KBD["/keyboard_input/input<br/>← llm_voice_agent<br/>→ llm_parser, grounding(fallback)"]
            T_VOICE["/voice_input/input<br/>← llm_voice_agent<br/>→ llm_parser, env_scan, static_env_report"]
            T_TEXT["/text_input<br/>← keyboard_input_node<br/>→ (待消费)"]
        end

        subgraph TOPIC_PARSE["解析链路 Topics"]
            T_PARSED["/parsed_command<br/>← llm_parser<br/>→ grounding_node"]
            T_GROUNDED["/grounded_goal<br/>← grounding_node<br/>→ ground_executor_node"]
        end

        subgraph TOPIC_WM["世界模型 Topics ⚠ 不匹配"]
            T_WM_OBJ["/world_model/objects<br/>← yolo, wm_dummy, wm_from_tf<br/>→ env_scan, static_env_report"]
            T_WM_ROI["/world_model/roi_objects<br/>← roi_color_detector<br/>→ grounding_node ⚠ 孤岛"]
        end

        subgraph TOPIC_VISION["视觉 Topics"]
            T_VISION["/vision_target<br/>DetectionResult<br/>← yolo → executor, env_scan"]
            T_ROI_VISION["/roi_vision_target<br/>DetectionResult<br/>← roi_color_detector → (待消费)"]
            T_ENV["/env_objects<br/>EnvObjectArray<br/>← env_scan, static_env → voice_agent, world_model"]
        end

        subgraph TOPIC_EXEC["执行 Topics"]
            T_PREVIEW["/executor/preview* (×5)<br/>← ground_executor → (UI/logger)"]
            T_CONFIRM["/executor/confirm Bool<br/>/executor/confirm_str String<br/>→ ground_executor"]
            T_DONE["/executor/done Bool<br/>→ executor_done_sayer"]
        end

        subgraph TOPIC_VOICE["语音 Topics"]
            T_QUERY["/speech_query<br/>← funasr → llm_voice_agent"]
            T_REPLY["/speech_reply<br/>← voice_agent, funasr, done_sayer<br/>→ funasr, tts"]
            T_TTS_SPEAK["/tts_speaking Bool<br/>← tts → funasr"]
            T_TTS_DONE["/tts/done Bool<br/>← tts → voice_agent"]
            T_TTS_INT["/tts/interrupt Bool<br/>← voice_agent, funasr → tts"]
        end

        subgraph TOPIC_SERVO["舵机控制 Topics ⚠ 多写者冲突"]
            T_SERVO_CTRL["/servo_controller<br/>← executor, voice_agent, face_follow,<br/>env_scan, gesture, grasp<br/>→ controller_manager"]
            T_BUS_SERVO["/ros_robot_controller/bus_servo/set_position<br/>← ServoManager, executor(直连)<br/>→ ros_robot_controller"]
            T_JOINT_STATE["~/joint_states<br/>/controller_manager/joint_states<br/>← controller_manager → executor"]
        end

        subgraph TOPIC_FACE["面部/社交 Topics"]
            T_FACE_CTRL["/face_follow/control String<br/>← voice, gesture, env, static_env<br/>→ face_follow"]
            T_FACE_STATUS["/face_follow/status String<br/>← face_follow → gesture, env"]
            T_GESTURE_CMD["/gesture/cmd String<br/>← voice_agent → gesture"]
        end
    end

    subgraph SERVICES["🔧 Service 调用关系"]
        S_IK_POSE["/kinematics/set_pose_target<br/>SetRobotPose.srv<br/>← executor, grasp<br/>服务端: kinematics IK .so"]
        S_IK_CURRENT["/kinematics/get_current_pose<br/>GetRobotPose.srv<br/>← yolo, object_pose_publisher, calibration<br/>服务端: kinematics IK .so"]
        S_CLEAR["/grounding/clear_memory<br/>Trigger.srv<br/>← 外部/调试<br/>服务端: grounding_node"]
        S_INIT["~/init_finish<br/>Trigger.srv<br/>服务端: controller_manager"]
        S_HW_INIT["/ros_robot_controller/init_finish<br/>Trigger.srv<br/>← controller_manager<br/>服务端: ros_robot_controller"]
        S_GET_SERVO["/bus_servo/get_state<br/>GetBusServoState.srv<br/>← ServoManager<br/>服务端: ros_robot_controller"]
    end

    subgraph ACTIONS["🎬 Action 服务端"]
        A_ARM["arm_controller/follow_joint_trajectory<br/>FollowJointTrajectory<br/>joint1-5<br/>服务端: controller_manager"]
        A_GRIP["gripper_controller/follow_joint_trajectory<br/>FollowJointTrajectory<br/>r_joint (servo 10)<br/>服务端: controller_manager"]
    end

    subgraph PROBLEMS["⚠ 已知问题"]
        P1["❌ /world_model/objects ↔ /world_model/roi_objects<br/>topic 名不匹配<br/>grounding 收不到 yolo/wm_dummy/wm_tf"]
        P2["⚠ /servo_controller 多写者<br/>6个节点同时可写<br/>无控制权仲裁"]
        P3["⚠ executor 直连 bus_servo/set_position<br/>绕过 controller_manager<br/>绕过 JointPositionController 限位"]
        P4["⚠ /text_input 无下游消费<br/>keyboard_input_node 输出孤岛"]
    end

    T_PARSED --> T_GROUNDED
    T_WM_OBJ -.->|"⚠ 不匹配"| T_WM_ROI
    T_VISION -.->|"可选"| T_GROUNDED
    T_REPLY --> T_QUERY
    T_TTS_SPEAK -.->|"gate"| T_QUERY
    P1 -.-> T_WM_OBJ
    P2 -.-> T_SERVO_CTRL
    P3 -.-> T_BUS_SERVO

    style S_IK_POSE fill:#d1ecf1,stroke:#0c5460
    style S_IK_CURRENT fill:#d1ecf1,stroke:#0c5460
    style T_WM_ROI fill:#fff3cd,stroke:#ffc107
    style T_SERVO_CTRL fill:#f8d7da,stroke:#dc3545
    style T_BUS_SERVO fill:#f8d7da,stroke:#dc3545
```

---

## 4. Teleop 与 RobotOps 未来架构图

```mermaid
graph TD
    subgraph TELEOP["🕹️ Teleop 输入层 P1"]
        KB_TELEOP["keyboard_teleop_node<br/>键盘→笛卡尔/关节速度"]
        GAMEPAD["gamepad_teleop_node<br/>手柄→笛卡尔/关节速度"]
        WEB_TELEOP["web_teleop_bridge<br/>WebSocket→ROS2 指令"]
        REMOTE["remote_control_adapter<br/>远程接管指令"]
    end

    subgraph ARBITRATION["🛡️ 控制权仲裁 P1"]
        ARBITER["control_arbiter_node<br/>全局控制状态管理"]
        ARB_STATE["/control/state String<br/>AUTO | MANUAL | PAUSED | ESTOP"]
        ARB_REQUEST["/control/request<br/>申请控制权"]
        ARB_ACK["/control/ack<br/>确权结果"]
        ARB_LOG["每次控制权切换<br/>发布 /robotops/arbitration_log"]
    end

    subgraph TASK_RUNTIME["📋 Task Runtime P0"]
        TASK_MGR["task_manager_node<br/>TaskContext 生命周期"]
        TASK_CTX["TaskContext<br/>task_id, user_command,<br/>parsed_command, target_object,<br/>selected_skill, execution_state,<br/>result, error_reason, timestamp"]
        TASK_STATE["/runtime/task_state<br/>task_id+state+progress"]
        TASK_EVENT["/runtime/task_event<br/>结构化事件流"]
    end

    subgraph SKILL_RUNTIME["⚙️ Skill Runtime P0"]
        SKILL_MGR["skill_manager_node<br/>SkillRegistry 注册+调度"]
        PICK_SKILL["pick_skill<br/>抓取序列<br/>hover→open→approach→close→lift"]
        PLACE_SKILL["place_skill<br/>放置序列<br/>hover→approach→open→lift"]
        MOVE_SKILL["move_skill<br/>移动到目标位姿"]
        HOME_SKILL["home_skill<br/>回到初始位姿"]
        ADAPTER["skill_adapter<br/>调用 IK / servo<br/>不直连硬件"]
    end

    subgraph ROBOTOPS["📊 RobotOps 日志/回传 P2"]
        LOGGER["task_logger_node<br/>/robotops/event_log<br/>→ SQLite 持久化"]
        MONITOR["state_monitor_node<br/>/robotops/state_summary<br/>实时状态汇总"]
        RECORDER["event_recorder<br/>事件时序记录<br/>/robotops/timeline"]
        DB["SQLite DB<br/>task_id 索引<br/>完整执行链路记录"]
    end

    subgraph DASHBOARD["📈 Dashboard / 回放 P2"]
        API["dashboard_api_node<br/>aiohttp REST API<br/>GET /api/tasks<br/>GET /api/task/:id<br/>GET /api/task/:id/snapshot"]
        REPLAY["replay_service<br/>任务轨迹回放<br/>图像+脉冲时序对齐"]
        EXPORT["data_exporter<br/>导出训练数据集格式<br/>VLA dataset builder"]
    end

    subgraph VERIFICATION["✅ 任务验证 P3"]
        VERIFY_NODE["verification_node<br/>/verification/result"]
        PRE_CHECK["抓取前检查<br/>目标是否存在<br/>/vision_target 查询"]
        POST_CHECK["抓取后检查<br/>目标是否消失<br/>夹爪ID10是否闭合<br/>servo是否到位"]
        TIMEOUT["超时检测<br/>预设时间超时→失败"]
        RETRY["retry_manager<br/>根据验证结果<br/>决定重试/跳过/报告"]
    end

    subgraph HW_LAYER["🦾 底层适配层 — 永不该修改"]
        IK_ADAPTER["ik_adapter<br/>封装 /kinematics/set_pose_target<br/>/kinematics/get_current_pose"]
        SERVO_ADAPTER["servo_adapter<br/>封装 /servo_controller<br/>不直连 bus_servo"]
        JOINT_STATE["joint_state_reader<br/>/controller_manager/joint_states"]
    end

    %% 连线
    KB_TELEOP --> ARB_REQUEST
    GAMEPAD --> ARB_REQUEST
    WEB_TELEOP --> ARB_REQUEST
    REMOTE --> ARB_REQUEST
    ARB_REQUEST --> ARBITER
    ARBITER --> ARB_STATE
    ARBITER --> ARB_ACK
    ARBITER --> ARB_LOG

    TASK_MGR --> TASK_CTX
    TASK_CTX --> TASK_STATE
    TASK_CTX --> TASK_EVENT

    TASK_CTX --> SKILL_MGR
    SKILL_MGR --> PICK_SKILL
    SKILL_MGR --> PLACE_SKILL
    SKILL_MGR --> MOVE_SKILL
    SKILL_MGR --> HOME_SKILL
    PICK_SKILL --> ADAPTER
    PLACE_SKILL --> ADAPTER
    MOVE_SKILL --> ADAPTER
    HOME_SKILL --> ADAPTER
    ARB_ACK -.->|"AUTO mode 确认"| ADAPTER

    TASK_EVENT --> LOGGER
    TASK_STATE --> MONITOR
    LOGGER --> RECORDER
    LOGGER --> DB
    RECORDER --> DB
    MONITOR --> API

    DB --> API
    DB --> REPLAY
    DB --> EXPORT

    TASK_CTX --> VERIFY_NODE
    VERIFY_NODE --> PRE_CHECK
    VERIFY_NODE --> POST_CHECK
    VERIFY_NODE --> TIMEOUT
    VERIFY_NODE --> RETRY
    RETRY -.->|"决定是否重试"| SKILL_MGR

    ADAPTER --> IK_ADAPTER
    ADAPTER --> SERVO_ADAPTER
    ADAPTER --> JOINT_STATE
    ARB_ACK -.->|"MANUAL mode"| KB_TELEOP
    ARB_ACK -.->|"MANUAL mode"| GAMEPAD

    style ARBITER fill:#fff3cd,stroke:#ffc107
    style IK_ADAPTER fill:#d1ecf1,stroke:#0c5460
    style SERVO_ADAPTER fill:#d1ecf1,stroke:#0c5460
    style JOINT_STATE fill:#d1ecf1,stroke:#0c5460
    style DB fill:#d4edda,stroke:#28a745
```

---

## 5. Sprint 1 重构目标图

```mermaid
graph TD
    subgraph SPRINT1_GOAL["🎯 Sprint 1 目标：建立 Robot Runtime v0.1 最小闭环"]
        GOAL_TEXT["从 用户指令 → task_id → skill → 执行 → 结果<br/>不破坏现有链路"]
    end

    subgraph EXISTING["📦 现有节点 — 最小改动"]
        subgraph EXIST_PARSE["现有 parser"]
            PARSER_NODE["llm_command_parser_node<br/>改动：无<br/>仍发布 /parsed_command"]
        end

        subgraph EXIST_GROUND["现有 grounding — FIX"]
            GROUND_FIX["grounding_node<br/>改动：launch文件remap<br/>/world_model/roi_objects<br/>→ /world_model/objects<br/>修复 topic 不匹配"]
        end

        subgraph EXIST_EXEC["现有 executor — 最小注入"]
            EXEC_INJECT["ground_executor_node<br/>改动：最小注入<br/>① 生成 task_id<br/>② 发布 /runtime/log<br/>③ 执行结果发布<br/>⚠ 不改变10步序列"]
        end

        subgraph EXIST_WM["现有世界模型"]
            YOLO_NODE["app_compatible_yolo_node<br/>改动：无<br/>/vision_target<br/>/world_model/objects"]
            WM_DUMMY["wm_dummy_pub<br/>改动：无<br/>/world_model/objects"]
            ROI_NODE["roi_color_detector_node<br/>改动：无<br/>/world_model/roi_objects"]
        end
    end

    subgraph NEW["🆕 新建 sketch_runtime 包 — Sprint 1"]
        subgraph RT_STATE["runtime_state_node"]
            RTS_INIT["RuntimeStateNode<br/>初始化+参数加载"]
            TASK_ID_GEN["task_id 生成<br/>UUID v4 + timestamp<br/>例: task_a1b2c3d4e5f6_1715900000"]
            STATE_TRACK["状态追踪<br/>parsed → grounded →<br/>executing → done/failed"]
            STATE_PUB["发布 /runtime/state<br/>String JSON<br/>{task_id,state,timestamp,...}"]
            EVENT_LOG["事件日志<br/>发布 /runtime/log<br/>String JSON<br/>{task_id,event,data,...}"]
        end

        subgraph SKILL_MGR["skill_manager 骨架"]
            SKILL_DICT["SkillManager.SKILLS<br/>静态字典映射<br/>'pick'→'pick_skill'<br/>'place'→'place_skill'<br/>'move'→'move_skill'"]
            SKILL_SELECT["select(intent)→skill_name<br/>纯语意层<br/>不连接控制回路"]
        end

        subgraph MSG_NEW["标准化消息"]
            TARGET_OBJ["TargetObject.msg<br/>(扩展 vision_interfaces)<br/>task_id, name, class_name,<br/>color, confidence,<br/>world_x,y,z,<br/>source, timestamp"]
        end
    end

    subgraph LAUNCH["🚀 Launch 集成"]
        LAUNCH_GROUND["ground_bringup.launch.py<br/>改动：增加remap<br/>world_model/roi_objects<br/>→ world_model/objects"]
        LAUNCH_RT["runtime_bringup.launch.py<br/>新建：启动<br/>runtime_state_node<br/>+ ground_executor_node<br/>+ grounding_node"]
    end

    subgraph TOPICS_NEW["📡 Sprint 1 新 Topics"]
        T_STATE["/runtime/state<br/>String JSON<br/>→ logger, dashboard(日后)"]
        T_LOG["/runtime/log<br/>String JSON<br/>→ logger(日后), replay(日后)"]
        T_EXEC_LOG["/runtime/execution_result<br/>String JSON<br/>← executor<br/>{task_id,success,reason,...}"]
    end

    subgraph VERIFICATION["🚫 Sprint 1 不做"]
        NO_SKILL["❌ 不重写 executor 动作序列"]
        NO_ARBITER["❌ 不引入控制权仲裁"]
        NO_DB["❌ 不引入持久化存储"]
        NO_IK["❌ 不修改 IK / servo_controller"]
        NO_VOICE["❌ 不重构语音/vision"]
    end

    %% 连线
    PARSER_NODE -->|"/parsed_command"| RTS_INIT
    GROUND_FIX -->|"/grounded_goal"| RTS_INIT
    EXEC_INJECT -->|"task_id+result"| T_EXEC_LOG

    RTS_INIT --> TASK_ID_GEN
    TASK_ID_GEN --> STATE_TRACK
    STATE_TRACK --> STATE_PUB --> T_STATE
    STATE_TRACK --> EVENT_LOG --> T_LOG

    SKILL_DICT --> SKILL_SELECT

    YOLO_NODE -->|"/world_model/objects"| GROUND_FIX
    WM_DUMMY -->|"/world_model/objects"| GROUND_FIX
    ROI_NODE -->|"/world_model/roi_objects"| GROUND_FIX

    T_STATE -->|"(日后)"| API["dashboard API"]
    T_LOG -->|"(日后)"| DB_STORE["SQLite"]

    style GROUND_FIX fill:#fff3cd,stroke:#ffc107
    style EXEC_INJECT fill:#fff3cd,stroke:#ffc107
    style RTS_INIT fill:#d4edda,stroke:#28a745
    style SKILL_SELECT fill:#d4edda,stroke:#28a745
    style TARGET_OBJ fill:#d4edda,stroke:#28a745
```

---

## 6. 感知链路重构 — Raw Detection → Stable World Model

> **现状**: `/world_model/roi_objects` 是原始检测流 — 同一物体在不同帧之间 class/color 跳变（cup red → cup black → cylinder red）。YOLO `/world_model/objects` 提供稳定语义类名但无颜色/可靠位姿。
> **目标**: YOLO + ROI 在 Grounding 之前融合 → StableObjectTracker 时序稳定 → 供 Grounding 消费。

```mermaid
graph TD
    subgraph CURRENT["🔴 旧架构 (Sprint 5 之前)"]
        CAM1["深度相机"]
        ROI1["roi_color_detector_node"]
        WM_RAW1["/world_model/roi_objects<br/>RAW JSON"]
        GND1["grounding_node"]
        RT1["Runtime"]
    end

    subgraph TARGET["🟢 目标架构 (Sprint 5)"]
        CAM2["深度相机"]
        YOLO2["yolo_node<br/>YOLOv8 语义检测"]
        ROI2["roi_color_detector_node<br/>ROI 颜色+位姿"]
        WM_YOLO["/world_model/objects<br/>YOLO JSON"]
        WM_RAW2["/world_model/roi_objects<br/>RAW JSON"]
        FUSION["perception_fusion_node<br/>空间匹配 + 融合"]
        WM_FUSED["/world_model/perception_objects<br/>Fused JSON"]
        TRACKER["StableObjectTracker<br/>═════════════<br/>Temporal Voting<br/>EMA Confidence Smoothing<br/>Object TTL (timeout)"]
        WM_STABLE["/world_model/stable_objects<br/>Stable JSON"]
        GND2["grounding_node<br/>(改用 stable_objects)"]
        RT2["Runtime → ActionExecutor → IK/Servo"]
    end

    CAM1 --> ROI1 --> WM_RAW1 --> GND1 --> RT1
    CAM2 --> YOLO2 --> WM_YOLO --> FUSION
    CAM2 --> ROI2 --> WM_RAW2 --> FUSION
    FUSION --> WM_FUSED --> TRACKER --> WM_STABLE --> GND2 --> RT2

    style CURRENT fill:#f8d7da,stroke:#dc3545
    style TARGET fill:#d4edda,stroke:#28a745
    style FUSION fill:#fff3cd,stroke:#ffc107
    style TRACKER fill:#d1ecf1,stroke:#0c5460
    style WM_STABLE fill:#fff3cd,stroke:#ffc107
```

**StableObjectTracker 核心逻辑**:
- 每帧接收 RAW detections → 与已知 track 匹配 (空间距离)
- 同一 track 的 class/color 做多数投票 (Temporal Voting, 窗口=20 帧)
- confidence 做 EMA 平滑 (α=0.2)
- 连续未检测 → TTL 超时移除 (3 秒)
- 输出 `/world_model/stable_objects`



---

## 7. Perception Fusion Policy

> **生效日期**: Sprint 5.4

### Fusion Pipeline

```
YOLO /world_model/objects           ROI /world_model/roi_objects
  (semantic class_name)               (color, pose.xyz, pose.rpy)
        │                                      │
        └──────────────┬───────────────────────┘
                       ▼
           perception_fusion_node
               │
               ▼
    /world_model/perception_objects
               │
               ▼
        StableObjectTracker
               │
               ▼
    /world_model/stable_objects
               │
               ▼
           grounding → Runtime
```

### Source Priority

| Source | Trusted For | Reason |
|--------|-------------|--------|
| YOLO   | semantic class_name | YOLOv8 provides stable COCO-class labels; not dependent on lighting |
| ROI    | color, pose.xyz, pose.rpy | LAB + geometric projection provides accurate world coordinates and yaw |
| ROI    | fallback class_name | Some objects (red cubes/blocks) are not in YOLO's vocabulary → ROI shape classifier as fallback |

**Note**: ROI class_name is lower trust than YOLO. ROI shape/color classification is known to be unstable across frames (Sprint 5.3 deferred). Strategic decision: YOLO semantic priority, ROI spatial/color priority.

### Fusion Rules

| Match | class_name | color | pose | source | Extra fields |
|-------|-----------|-------|------|--------|-------------|
| YOLO + ROI (dist < 0.06m) | YOLO | ROI | ROI (xyz+rpy) | `yolo_roi_fused` | `yolo_class`, `roi_class`, `match_distance` |
| ROI only | ROI | ROI | ROI | `roi_only` | — |
| YOLO only | YOLO | `unknown` | YOLO | `yolo_only` | — |

### Design Rationale

- **YOLO semantic priority**: YOLO correctly identifies "cup" while ROI may call it "cylinder" or "cube" due to shape classification instability.
- **ROI spatial priority**: ROI's `_pix_to_world()` via static `transform.yaml` provides reliable world coordinates without IK dependency. YOLO's `_pix_to_world_on_plane()` requires a live `/kinematics/get_current_pose` call.
- **ROI-only fallback kept enabled**: Red cubes/blocks are not in YOLO's COCO whitelist. ROI shape classifier remains the only detector for these objects. ROI color is essential for color-based queries ("red cube").
- **StableObjectTracker runs after fusion**: Temporal voting and EMA smoothing occur on fused output, not on raw ROI. This prevents stabilizing wrong ROI-only labels before YOLO semantic correction is applied.

---

## 图例说明

| 颜色 | 含义 |
|------|------|
| 🔵 蓝色边框 | 底层硬件/驱动 — 永不应修改 |
| 🟡 黄色填充 | 需要修复/重构的模块 (Sprint 1) |
| 🔴 红色填充 | 当前高风险区域 (硬编码/冲突) |
| 🟢 绿色填充 | Sprint 1 新建模块 |
| ⚠ 标记 | 已知 Bug / 安全风险 |
