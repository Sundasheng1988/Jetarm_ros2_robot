"""place_manager 的集中配置。"""

import os
from pathlib import Path


# 节点
PLACE_MANAGER_NODE_NAME = 'place_manager'
GOTO_PLACE_NODE_NAME = 'goto_place'

# 坐标系与 Topic
MAP_FRAME = 'map'
AMCL_POSE_TOPIC = '/amcl_pose'
ODOM_TOPIC = '/odom_combined'

# Service
SAVE_PLACE_SERVICE = '/save_place'
DELETE_PLACE_SERVICE = '/delete_place'
LIST_PLACES_SERVICE = '/list_places'
GET_PLACE_SERVICE = '/get_place'
GOTO_PLACE_SERVICE = '/goto_place'
# 取消当前 Nav2 导航目标（由 /cancel_navigation 服务暴露，标准 Trigger）。
CANCEL_NAVIGATION_SERVICE = '/cancel_navigation'

# Nav2 Action
NAV2_NAVIGATE_TO_POSE_ACTION = '/navigate_to_pose'

# 超时（均可在 GotoPlaceNode 中通过 ROS 参数覆盖）
NAV2_WAIT_FOR_SERVER_TIMEOUT = 5.0
NAV2_ACTION_TIMEOUT = 180.0
NAV2_CANCEL_TIMEOUT = 5.0
NAV2_FEEDBACK_LOG_PERIOD = 5.0

# 到达验证默认值
ARRIVAL_VERIFY_TIMEOUT = 5.0
ARRIVAL_POSITION_TOLERANCE = 0.30       # m
ARRIVAL_YAW_TOLERANCE = 0.35            # rad，约 20°
ARRIVAL_LINEAR_SPEED_TOLERANCE = 0.03   # m/s
ARRIVAL_ANGULAR_SPEED_TOLERANCE = 0.05  # rad/s
ARRIVAL_STATE_MAX_AGE = 2.0             # s
ARRIVAL_STABLE_DURATION = 0.50           # s

# PlaceManager 保存位姿时允许的最大消息年龄
AMCL_POSE_MAX_AGE = 2.0

# 默认运行时 places.yaml，可通过 places_file 参数覆盖
DEFAULT_PLACES_FILE = os.path.join(
    str(Path.home()),
    'ros2_ws',
    'config',
    'places.yaml',
)
