#!/usr/bin/env bash
# JetArm ROS 2 PC 环境统一入口（必须 source，不要直接执行）
#
# 用法：
#   source ~/ros2_ws/scripts/ros_env_pc.sh voice    # PC-only Voice/Nav 回归
#   source ~/ros2_ws/scripts/ros_env_pc.sh robot    # PC <-> Orin 跨机
#   source ~/ros2_ws/scripts/ros_env_pc.sh status   # 只查看当前环境
#
# voice: DOMAIN=23 + Cyclone + ROS_LOCALHOST_ONLY=1 + 不加载任何 Cyclone XML
#        完全不依赖 eno1 / wlo1，换网络不影响。
# robot: DOMAIN=23 + Cyclone + ROS_LOCALHOST_ONLY=0
#        仅当 eno1 实际持有 192.168.100.x 地址且 XML 存在时才加载 pc_camera_eno1.xml；
#        否则保持 CYCLONEDDS_URI unset 并打印 WARNING，绝不静默加载错误 XML。

(return 0 2>/dev/null) || {
  echo "ERROR: 本脚本必须 source 使用，不能直接执行："
  echo "  source ~/ros2_ws/scripts/ros_env_pc.sh voice|robot|status"
  exit 1
}

WS_HOME="$HOME/ros2_ws"
MODE="${1:-}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_HOME/install/setup.bash"

active_iface() {
  ip route 2>/dev/null | awk '/^default/ {print $5; exit}'
}

print_status() {
  echo "[ROS ENV]"
  echo "  mode=${ROS_ENV_MODE:-<unchanged>}"
  echo "  domain=${ROS_DOMAIN_ID:-<unset>}"
  echo "  rmw=${RMW_IMPLEMENTATION:-<unset, 默认将回退 rmw_fastrtps_cpp>}"
  echo "  localhost_only=${ROS_LOCALHOST_ONLY:-<unset>}"
  echo "  cyclonedds_uri=${CYCLONEDDS_URI:-<unset>}"
  echo "  active_iface=$(active_iface)"
}

case "$MODE" in
  status)
    print_status
    return 0
    ;;
  voice|robot)
    ;;
  "")
    echo "ERROR: 缺少模式参数。用法："
    echo "  source ~/ros2_ws/scripts/ros_env_pc.sh voice|robot|status"
    print_status
    return 1
    ;;
  *)
    echo "ERROR: 未知模式 '$MODE'（仅支持 voice / robot / status）"
    return 1
    ;;
esac

# --- source ROS Humble + workspace ---
if [ ! -r "$ROS_SETUP" ]; then
  echo "ERROR: 找不到 $ROS_SETUP"
  return 1
fi
if [ ! -r "$WS_SETUP" ]; then
  echo "ERROR: 找不到 $WS_SETUP（工作区尚未构建）"
  return 1
fi
source "$ROS_SETUP"
source "$WS_SETUP"

# --- 公共基线（在 source 之后导出，防止被 setup 链覆盖）---
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ "$MODE" = "voice" ]; then
  # PC-only：只走 loopback，不依赖任何物理网卡 / Cyclone XML
  export ROS_LOCALHOST_ONLY=1
  unset CYCLONEDDS_URI
else
  # robot：只有 eno1 有线机器人链路真实在线时才允许绑定 eno1 XML
  export ROS_LOCALHOST_ONLY=0
  ENO1_XML="$WS_HOME/config/cyclonedds/pc_camera_eno1.xml"
  if ip link show dev eno1 >/dev/null 2>&1 \
    && [ "$(cat /sys/class/net/eno1/carrier 2>/dev/null)" = "1" ] \
    && ip -4 addr show dev eno1 2>/dev/null | grep -q 'inet 192\.168\.100\.'; then
    if [ -r "$ENO1_XML" ]; then
      export CYCLONEDDS_URI="file://$ENO1_XML"
    else
      unset CYCLONEDDS_URI
      echo "WARNING: eno1 在线但缺少 $ENO1_XML"
      echo "         CYCLONEDDS_URI 保持 unset，使用 Cyclone 默认 discovery。"
    fi
  else
    unset CYCLONEDDS_URI
    echo "WARNING: 未检测到 eno1 有线机器人链路（eno1 down 或无 192.168.100.x 地址）。"
    echo "         CYCLONEDDS_URI 保持 unset，当前网络下通常无法发现 Orin 节点。"
    echo "         当前活动网卡: $(active_iface)"
    echo "         如 Orin 改经 Wi-Fi/hotspot 连接，请先核对 pc_camera_wlo1.xml 内"
    echo "         的 Peer 地址是否仍有效，再手动 export；本脚本不会静默加载它。"
  fi
fi

export ROS_ENV_MODE="$MODE"
print_status
echo "  提示: 修改 RMW/DDS 环境后，已运行的 ROS 2 进程不会自动切换，需重启相关进程。"
