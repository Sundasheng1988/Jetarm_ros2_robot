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

`/goto_place` 仍是长时 Service，因此调用端会等待导航结束才收到响应。该版本保持
现有接口兼容；后续若接入 Runtime 的取消、抢占和任务状态，建议再将语义导航接口升级
为自定义 Action。
