#!/usr/bin/env bash
# JetArm ROS 2 PC 环境统一入口（必须 source，不要直接执行）
#
# 用法：
#   source ~/ros2_ws/scripts/ros_env_pc.sh voice    # PC-only Voice/Nav 回归
#   source ~/ros2_ws/scripts/ros_env_pc.sh robot    # PC <-> Orin 跨机
#   source ~/ros2_ws/scripts/ros_env_pc.sh status   # 只查看当前环境
#
# voice: DOMAIN=23 + Cyclone + ROS_LOCALHOST_ONLY=1
#        加载 pc_localhost.xml（绑定 lo，MaxAutoParticipantIndex=120）
#        完全不依赖 eno1 / wlo1，换网络不影响；XML 缺失即 ERROR 退出。
# robot: DOMAIN=23 + Cyclone + ROS_LOCALHOST_ONLY=0
#        优先 eno1 有线机器人网络（192.168.100.x -> pc_camera_eno1.xml）；
#        机器人经 Wi-Fi/hotspot 连接时用 wlo1（172.20.10.x -> pc_camera_wlo1.xml）；
#        XML 缺失或网络不明确 -> ERROR + return 1（fail-closed），
#        绝不 WARNING 后回退 Cyclone 默认 discovery。
# 公共：ROS2CLI_DISABLE_DAEMON=1，避免 stale CLI daemon 复用旧 DDS 环境。
# 核心语义：模式切换采用 preflight -> commit；任何 preflight 失败均不修改
# 当前 shell 的 ROS/DDS mode 环境（validate first, commit once）。

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

# ================= PHASE 1: PREFLIGHT =================
# 本阶段只读检查并计算目标配置（SELECTED_* 普通变量）；
# 绝不 export/unset ROS/DDS mode 环境变量，绝不 source setup。

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

# --- setup 文件只做可读检查，preflight 全部通过前不 source ---
if [ ! -r "$ROS_SETUP" ]; then
  echo "ERROR: 找不到 $ROS_SETUP"
  return 1
fi
if [ ! -r "$WS_SETUP" ]; then
  echo "ERROR: 找不到 $WS_SETUP（工作区尚未构建）"
  return 1
fi

SELECTED_LOCALHOST_ONLY=""
SELECTED_CYCLONEDDS_URI=""
SELECTED_NETWORK=""

if [ "$MODE" = "voice" ]; then
  # PC-only：只走 loopback，绑定 pc_localhost.xml
  LOCAL_XML="$WS_HOME/config/cyclonedds/pc_localhost.xml"
  if [ ! -r "$LOCAL_XML" ]; then
    echo "ERROR: 缺少 $LOCAL_XML"
    return 1
  fi
  SELECTED_LOCALHOST_ONLY=1
  SELECTED_CYCLONEDDS_URI="file://$LOCAL_XML"
else
  # robot：优先使用 eno1 有线机器人网络；
  # 如果机器人通过 Wi-Fi / hotspot 连接，则自动使用 wlo1。
  ENO1_XML="$WS_HOME/config/cyclonedds/pc_camera_eno1.xml"
  WLO1_XML="$WS_HOME/config/cyclonedds/pc_camera_wlo1.xml"

  if ip link show dev eno1 >/dev/null 2>&1 \
    && [ "$(cat /sys/class/net/eno1/carrier 2>/dev/null)" = "1" ] \
    && ip -4 addr show dev eno1 2>/dev/null \
       | grep -q 'inet 192\.168\.100\.'; then

    if [ -r "$ENO1_XML" ]; then
      SELECTED_LOCALHOST_ONLY=0
      SELECTED_CYCLONEDDS_URI="file://$ENO1_XML"
      SELECTED_NETWORK="eno1 / 192.168.100.x"
    else
      echo "ERROR: eno1 在线，但缺少 $ENO1_XML"
      return 1
    fi

  elif ip link show dev wlo1 >/dev/null 2>&1 \
    && ip -4 addr show dev wlo1 2>/dev/null \
       | grep -q 'inet 172\.20\.10\.'; then

    if [ -r "$WLO1_XML" ]; then
      SELECTED_LOCALHOST_ONLY=0
      SELECTED_CYCLONEDDS_URI="file://$WLO1_XML"
      SELECTED_NETWORK="wlo1 / 172.20.10.x"
    else
      echo "ERROR: wlo1 在线，但缺少 $WLO1_XML"
      return 1
    fi

  else
    echo "ERROR: 未检测到机器人网络。"
    echo "       eno1: 需要 192.168.100.x"
    echo "       wlo1: 需要 172.20.10.x"
    echo "       当前活动网卡: $(active_iface)"
    return 1
  fi
fi

# ================= PHASE 2: COMMIT =================
# 所有 preflight 已通过；从这里开始才允许改变当前 shell 环境。

source "$ROS_SETUP"
source "$WS_SETUP"

# --- 公共基线（在 source 之后导出，防止被 setup 链覆盖）---
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS2CLI_DISABLE_DAEMON=1

export ROS_LOCALHOST_ONLY="$SELECTED_LOCALHOST_ONLY"
export CYCLONEDDS_URI="$SELECTED_CYCLONEDDS_URI"

export ROS_ENV_MODE="$MODE"

if [ "$MODE" = "robot" ] && [ -n "$SELECTED_NETWORK" ]; then
  echo "ROS robot network: $SELECTED_NETWORK"
fi
print_status
echo "  提示: 修改 RMW/DDS 环境后，已运行的 ROS 2 进程不会自动切换，需重启相关进程。"
