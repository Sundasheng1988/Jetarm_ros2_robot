# Servo Controller 无法控制机械臂故障排查记录

## 问题现象

系统运行正常。

ROS2 话题存在：

```bash
ros2 topic list
```

可以看到：

```text
/servo_controller
/ros_robot_controller/bus_servo/set_position
```

发送控制指令：

```bash
ros2 topic pub --once /servo_controller servo_controller_msgs/msg/ServosPosition \
"{duration: 1.0, position_unit: pulse, position: [{id: 10, position: 600.0}]}"
```

或者：

```bash
ros2 topic pub --once /ros_robot_controller/bus_servo/set_position \
ros_robot_controller_msgs/msg/ServosPosition \
"{duration: 1.0, position: [{id: 10, position: 600}]}"
```

均能够正常发布。

但是：

* 舵机无动作
* 夹爪无动作
* 无报错

---

# 第一步：确认话题链路

检查：

```bash
ros2 topic info /servo_controller -v
```

确认存在订阅者：

```text
Subscription count > 0
```

检查：

```bash
ros2 topic info /ros_robot_controller/bus_servo/set_position -v
```

确认底层也有订阅者。

如果没有订阅者：

说明控制节点未启动。

---

# 第二步：确认消息到达底层

监听：

```bash
ros2 topic echo /ros_robot_controller/bus_servo/set_position
```

再次发送控制命令。

如果能够看到：

```text
id: 10
position: 600
```

说明：

```text
/servo_controller
↓
servo_manager
↓
ros_robot_controller
```

链路正常。

问题已经进入硬件控制层。

---

# 第三步：检查控制节点是否重复启动

检查：

```bash
ros2 node list | grep -E "controller|servo|manager|robot"
```

异常情况：

```text
/controller_manager
/controller_manager

/servo_manager
/servo_manager

/ros_robot_controller
/ros_robot_controller
```

或者：

```text
/controller_manager ×3
/servo_manager ×3
```

说明：

系统存在多套控制节点。

---

# 根因

通过：

```bash
ps aux | grep -E "launch|bringup|ros2"
```

发现：

```text
ros2 launch bringup bringup.launch.py
```

被启动了两次。

例如：

```text
PID 592
bringup.launch.py

PID 5992
bringup.launch.py
```

导致：

```text
ros_robot_controller ×2

servo_controller ×2

grasp ×2
```

同时存在。

控制链混乱。

最终表现：

```text
话题正常
消息正常

舵机不动
```

---

# 解决方案

## 1 清理所有控制节点

```bash
pkill -9 -f "bringup.launch.py"

pkill -9 -f "ros_robot_controller"

pkill -9 -f "servo_controller"

pkill -9 -f "controller_manager"

pkill -9 -f "servo_manager"

pkill -9 -f "grasp"
```

---

## 2 清理 ROS 图缓存

```bash
ros2 daemon stop

ros2 daemon start
```

确认：

```bash
ros2 node list
```

节点已经消失。

---

## 3 重新启动一套系统

```bash
cd ~/ros2_ws

source install/setup.bash

ros2 launch bringup bringup.launch.py
```

注意：

只启动一次。

不要重复执行。

---

## 4 验证

检查：

```bash
ros2 node list | grep -E "controller|servo|manager|robot"
```

正常情况：

```text
/controller_manager

/servo_manager

/ros_robot_controller
```

每个节点仅出现一次。

---

# 验证控制恢复

测试夹爪：

打开：

```bash
ros2 topic pub --once /servo_controller servo_controller_msgs/msg/ServosPosition \
"{duration: 1.0, position_unit: pulse, position: [{id: 10, position: 600.0}]}"
```

关闭：

```bash
ros2 topic pub --once /servo_controller servo_controller_msgs/msg/ServosPosition \
"{duration: 1.0, position_unit: pulse, position: [{id: 10, position: 200.0}]}"
```

成功恢复。

---

# 经验总结

当出现：

```text
Servo 指令发送成功

Topic 正常

无报错

机械臂不动作
```

首先检查：

```bash
ros2 node list | grep -E "controller|servo|manager|robot"
```

确认是否存在：

```text
重复的 ros_robot_controller

重复的 servo_manager

重复的 controller_manager
```

这是 JetArm 系统最容易忽略的故障之一。

优先级高于：

* IK 问题
* Topic 问题
* 消息格式问题
* 舵机参数问题
* 代码问题
