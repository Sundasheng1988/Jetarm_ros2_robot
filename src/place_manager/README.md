# Semantic Navigation v0.2

本目录是原 `place_manager` 7 个 Python 文件的更新版，可覆盖到现有 ROS 2
包的 `place_manager/` Python 模块目录。自定义 `srv/` 接口保持不变。

## 本次修复

- 使用 `GoalStatus.STATUS_*` 判断 Nav2 终态。
- 本地导航超时后主动请求取消 Nav2 目标。
- 发送响应迟到时自动取消可能被接受的目标。
- 移除 Service callback 中的 `spin_until_future_complete()`。
- `GotoPlaceNode` 改用 `MultiThreadedExecutor` 和独立 callback group。
- 读取地点时检查 YAML 文件变化；新保存的地点无需重启导航节点。
- 输出 `distance_remaining` 与 `number_of_recoveries`。
- Nav2 返回成功后，用 `/amcl_pose` 和 `/odom_combined` 验证位置、朝向和停车状态。
- 跳过 YAML 中无法转成有限浮点数的异常条目。
- 修复相对 `places_file` 路径的父目录创建。
- 保存地点前检查 AMCL frame 与消息新鲜度。

## 额外依赖

`goto_place_node.py` 新增了 `nav_msgs/Odometry`，请确认 `package.xml` 中包含：

```xml
<depend>nav_msgs</depend>
<depend>action_msgs</depend>
```

## 关键默认参数

```yaml
goto_place:
  ros__parameters:
    odom_topic: /odom_combined
    nav2_action_timeout: 180.0
    nav2_cancel_timeout: 5.0
    arrival_position_tolerance: 0.30
    arrival_yaw_tolerance: 0.35
    arrival_linear_speed_tolerance: 0.03
    arrival_angular_speed_tolerance: 0.05
    arrival_verify_timeout: 5.0
```

首次实机测试前，请先确认机器人实际里程计 Topic 仍为 `/odom_combined`。如果不同，
通过 `odom_topic` 参数覆盖。

## 运行约束

`/goto_place` 仍是长时 Service，因此调用端会等待导航结束才收到响应。取消已通过
独立的 `/cancel_navigation`（`std_srvs/srv/Trigger`）服务实现：它在 `goto_place_node`
的 Reentrant 回调组中运行，可在 `/goto_place` 阻塞期间并发取消当前 Nav2 目标，
无需把语义导航接口升级为自定义 Action。抢占（新目标顶替旧目标）当前**不支持**：
进行中收到第二个导航会以 `BUSY` 拒绝。

## 语音命名地点导航（phase 1）

在 `goto_place_node` 之上新增了两个组件，构成语音导航闭环：

- `navigation_executor_node`：消费 `/parsed_command` 中的
  `navigate_to_place` / `cancel_navigation` 动作，路由到 `/goto_place`，并把
  结果发布到 `/runtime/execution_result`。同一时刻只允许一个导航任务，进行中
  收到第二个以 `BUSY` 拒绝（不抢占）；异步调用，定时器轮询 Future，不在回调里阻塞。
- `/cancel_navigation`（`std_srvs/srv/Trigger`）：`goto_place_node` 新增的取消
  服务，运行在独立 Reentrant 回调组中，保证 `/goto_place` 阻塞回调期间仍可取消。
  无活动导航时幂等返回成功。

启动方式：

```bash
# 独立启动导航栈（goto_place_node + navigation_executor_node + executor_done_sayer）
ros2 launch place_manager navigation.launch.py

# 与 voice_stack.launch.py 同跑时关闭本 launch 自带的 done_sayer，避免重复播报
ros2 launch place_manager navigation.launch.py launch_done_sayer:=false

# 自定义地点文件
ros2 launch place_manager navigation.launch.py places_file:=/path/to/places.yaml
```

完整闭环（语音输入 → Rebecca 确认 → Parser → 导航执行 → 结果播报）还需
`voice_stack.launch.py`（ASR + Rebecca + llm_command_parser）与 Nav2 bringup。
本 launch **不**启动 Nav2，**不**发布 `/cmd_vel`。

### 安全边界

- phase 1 只导航到 `places.yaml` 中已保存的命名地点，不接受任意坐标。
- Rebecca / Parser / LLM 不发布 `/cmd_vel`、不生成坐标、不绕过 `place_manager`、
  不直接控 Nav2。
- “停止说话 / 安静”（只中止 LLM/TTS）与“停止移动 / 取消导航”（取消 Nav2 目标）
  严格分离。

详见 `docs/topic_service_map.md` 2.6 节与 `docs/runtime_task_schema.md` 8.6 节。
