# 2026-08-04 Dev Log

## RPLIDAR 串口故障排查

- 停止 `start_app_node.service` 后，`/dev/ttyUSB0` 的异常映射仍然存在。
- 查明内核识别到的 RPLIDAR 串口为：
  - 芯片：CP2102
  - VID/PID：`10c4:ea60`
  - 驱动：`cp210x`
  - 主次设备号：`188:0`
- 查明 `/dev/ttyUSB0` 实际是软链接：
  ```text
  /dev/ttyUSB0 -> ttyCH341USB0
  ```
- `ttyCH341USB0` 主次设备号为 `169:0`，由 `python3` 进程占用。
- 确认旧规则 `/etc/udev/rules.d/99-ttyACM0.rules` 为部分 USB 路径创建了 `ttyUSB0` 软链接，造成设备名冲突。

## 临时恢复与验证

- 创建临时雷达设备节点：
  ```text
  /dev/rplidar_cp2102
  ```
- 设备节点参数：
  ```text
  crw-rw---- root:dialout 188,0
  ```
- 使用以下配置成功启动 RPLIDAR：
  ```text
  serial_port=/dev/rplidar_cp2102
  serial_baudrate=115200
  ```
- 雷达启动信息：
  - 型号：RPLIDAR A1
  - 固件版本：`1.29`
  - 硬件版本：`7`
  - 健康状态：`OK`
  - 扫描模式：`Sensitivity`
  - 采样率：`8 kHz`
  - 最大距离：`12 m`
  - 节点报告扫描频率：`10 Hz`

## 底盘返修后运行验证

- 在瓷砖地面进行了低速原地旋转测试。
- 逆时针测试使用 `angular.z=-0.15 rad/s`，持续约 `25.1 s`。
- 顺时针测试使用 `angular.z=+0.15 rad/s`，持续约 `24.3 s`。
- 两个方向均可连续运行。
- 旋转过程中，激光点云与地图边界保持整体贴合。
- 未复现此前旋转后的激光点云整体偏转或地图错位。

## AMCL 定位验证

- 使用静态地图启动 AMCL：

  ```bash
  ros2 launch nav2_bringup localization_launch.py \
    map:=/home/ubuntu/ros2_ws/maps/home_map_clean_02_260705.yaml \
    use_sim_time:=false \
    params_file:=/home/ubuntu/ros2_ws/config/nav2_amcl_params.yaml
  ```

- 在 RViz 中设置初始位姿后，激光点云与静态地图保持整体贴合。
- 机器人完成直行、转弯、通过门口及跨房间移动。
- 运行过程中未重新设置 AMCL 初始位姿。
- 运行结束后未出现此前约 `90°` 的整体角度偏转。

## Nav2 启动

- 使用已有的 `/home/ubuntu/ros2_ws/config/nav2_params.yaml` 启动导航栈：

  ```bash
  ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    autostart:=true \
    params_file:=/home/ubuntu/ros2_ws/config/nav2_params.yaml
  ```

- Nav2 能够生成穿过房间与客厅之间门口的全局路径。
- 机器人能够根据 RViz 中设置的目标点执行自主导航。

## Nav2 多目标导航测试

- 连续设置了 6 个导航目标：
  1. 房间内第一个点。
  2. 离开房间后到达客厅第一个点。
  3. 客厅第二个短距离目标点。
  4. 靠近墙边并需要转向超过 `90°` 的目标点。
  5. 返回房间的目标点。
  6. 返回最初位置区域的目标点。
- 6 个目标中，5 个出现 `Goal succeeded`。
- 客厅第二个短距离目标未出现 `Goal succeeded`，随后被新目标抢占。
- 导航日志中共出现 10 次 `Failed to make progress`。
- 恢复过程中出现过代价地图清除、Spin 和 BackUp 行为。
- 一次 Spin 恢复要求旋转 `1.57 rad`，并在 `10 s` 后超时。
- 机器人最终完成跨房间往返并返回最初位置区域。
- 全程未重新设置 AMCL 初始位姿。

## 当日结论

- RPLIDAR 硬件、供电、USB 通信和波特率正常。
- 雷达无法启动的原因是旧 udev 规则造成 `/dev/ttyUSB0` 设备名冲突。
- 雷达通过临时设备节点恢复运行。
- 底盘完成顺、逆时针低速原地旋转测试。
- AMCL 完成跨房间连续定位测试。
- Nav2 完成跨房间多目标导航测试。
- 导航测试中存在 `Failed to make progress` 和恢复行为。
- 永久 udev 设备映射尚未修复。
