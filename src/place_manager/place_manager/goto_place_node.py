"""语义地点导航节点。

保持原有 ``/goto_place`` Service 接口；内部使用 MultiThreadedExecutor，
避免在 Service callback 中嵌套 spin。Nav2 成功后还会用 AMCL 与里程计
独立检查位置、朝向和停车状态。
"""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from place_manager.config import (
    AMCL_POSE_TOPIC,
    ARRIVAL_ANGULAR_SPEED_TOLERANCE,
    ARRIVAL_LINEAR_SPEED_TOLERANCE,
    ARRIVAL_POSITION_TOLERANCE,
    ARRIVAL_STABLE_DURATION,
    ARRIVAL_STATE_MAX_AGE,
    ARRIVAL_VERIFY_TIMEOUT,
    ARRIVAL_YAW_TOLERANCE,
    CANCEL_NAVIGATION_SERVICE,
    DEFAULT_PLACES_FILE,
    GOTO_PLACE_NODE_NAME,
    GOTO_PLACE_SERVICE,
    MAP_FRAME,
    NAV2_ACTION_TIMEOUT,
    NAV2_CANCEL_TIMEOUT,
    NAV2_FEEDBACK_LOG_PERIOD,
    NAV2_WAIT_FOR_SERVER_TIMEOUT,
    ODOM_TOPIC,
)
from place_manager.nav_client import NavClient
from place_manager.place_store import PlaceStore
from place_manager.pose_utils import normalize_angle, quaternion_to_yaw
from place_manager.srv import GotoPlace


class GotoPlaceNode(Node):
    """地点名称 -> Nav2 -> 独立到达验证。"""

    def __init__(self, places_file: str = DEFAULT_PLACES_FILE):
        super().__init__(GOTO_PLACE_NODE_NAME)

        self.declare_parameter('places_file', places_file)
        self.declare_parameter('odom_topic', ODOM_TOPIC)
        self.declare_parameter(
            'nav2_wait_for_server_timeout', NAV2_WAIT_FOR_SERVER_TIMEOUT,
        )
        self.declare_parameter('nav2_action_timeout', NAV2_ACTION_TIMEOUT)
        self.declare_parameter('nav2_cancel_timeout', NAV2_CANCEL_TIMEOUT)
        self.declare_parameter(
            'nav2_feedback_log_period', NAV2_FEEDBACK_LOG_PERIOD,
        )
        self.declare_parameter(
            'arrival_verify_timeout', ARRIVAL_VERIFY_TIMEOUT,
        )
        self.declare_parameter(
            'arrival_position_tolerance', ARRIVAL_POSITION_TOLERANCE,
        )
        self.declare_parameter(
            'arrival_yaw_tolerance', ARRIVAL_YAW_TOLERANCE,
        )
        self.declare_parameter(
            'arrival_linear_speed_tolerance',
            ARRIVAL_LINEAR_SPEED_TOLERANCE,
        )
        self.declare_parameter(
            'arrival_angular_speed_tolerance',
            ARRIVAL_ANGULAR_SPEED_TOLERANCE,
        )
        self.declare_parameter(
            'arrival_state_max_age', ARRIVAL_STATE_MAX_AGE,
        )
        self.declare_parameter(
            'arrival_stable_duration', ARRIVAL_STABLE_DURATION,
        )

        self._store = PlaceStore(
            str(self.get_parameter('places_file').value)
        )

        # Service 同一时间只处理一个导航请求；Action 与状态订阅可并行。
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self._state_group = ReentrantCallbackGroup()
        # 取消服务必须能在 /goto_place 阻塞回调执行期间并发响应，因此使用
        # 独立的 Reentrant 组，并由 main() 的 MultiThreadedExecutor 驱动。
        self._cancel_group = ReentrantCallbackGroup()

        self._nav_client = NavClient(
            self,
            callback_group=self._action_group,
            wait_for_server_timeout=float(
                self.get_parameter('nav2_wait_for_server_timeout').value
            ),
            action_timeout=float(
                self.get_parameter('nav2_action_timeout').value
            ),
            cancel_timeout=float(
                self.get_parameter('nav2_cancel_timeout').value
            ),
            feedback_log_period=float(
                self.get_parameter('nav2_feedback_log_period').value
            ),
        )

        self._state_lock = threading.Lock()
        self._latest_pose: Optional[Tuple[float, float, float, float, str]] = None
        self._latest_velocity: Optional[Tuple[float, float, float]] = None

        # 从进入 NavClient.goto() 到其返回期间保持置位。
        # 用于处理“取消指令早于 Nav2 goal handle 创建”的短暂竞态。
        self._navigation_in_progress = threading.Event()

        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            AMCL_POSE_TOPIC,
            self._pose_callback,
            10,
            callback_group=self._state_group,
        )
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            20,
            callback_group=self._state_group,
        )
        self._srv_goto = self.create_service(
            GotoPlace,
            GOTO_PLACE_SERVICE,
            self._handle_goto_place,
            callback_group=self._service_group,
        )
        self._srv_cancel = self.create_service(
            Trigger,
            CANCEL_NAVIGATION_SERVICE,
            self._handle_cancel_navigation,
            callback_group=self._cancel_group,
        )

        self.get_logger().info(
            f'GotoPlaceNode 启动: places={self._store.file_path}, '
            f'odom={odom_topic}, service={GOTO_PLACE_SERVICE}, '
            f'cancel={CANCEL_NAVIGATION_SERVICE}'
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        frame_id = msg.header.frame_id or MAP_FRAME
        with self._state_lock:
            self._latest_pose = (
                float(pose.position.x),
                float(pose.position.y),
                float(yaw),
                time.monotonic(),
                frame_id,
            )

    def _odom_callback(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        linear_speed = math.hypot(float(linear.x), float(linear.y))
        with self._state_lock:
            self._latest_velocity = (
                linear_speed,
                abs(float(angular.z)),
                time.monotonic(),
            )

    def _arrival_state(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
    ) -> Tuple[bool, str]:
        now = time.monotonic()
        max_age = float(self.get_parameter('arrival_state_max_age').value)

        with self._state_lock:
            pose = self._latest_pose
            velocity = self._latest_velocity

        if pose is None:
            return False, '尚未收到 /amcl_pose'
        if velocity is None:
            return False, '尚未收到里程计速度'

        x, y, yaw, pose_time, frame_id = pose
        linear_speed, angular_speed, velocity_time = velocity
        if frame_id != MAP_FRAME:
            return False, f'/amcl_pose frame_id={frame_id}，期望 {MAP_FRAME}'
        if now - pose_time > max_age:
            return False, f'/amcl_pose 已过期（{now - pose_time:.2f}s）'
        if now - velocity_time > max_age:
            return False, f'里程计速度已过期（{now - velocity_time:.2f}s）'

        position_error = math.hypot(x - target_x, y - target_y)
        yaw_error = abs(normalize_angle(yaw - target_yaw))
        position_tolerance = float(
            self.get_parameter('arrival_position_tolerance').value
        )
        yaw_tolerance = float(
            self.get_parameter('arrival_yaw_tolerance').value
        )
        linear_tolerance = float(
            self.get_parameter('arrival_linear_speed_tolerance').value
        )
        angular_tolerance = float(
            self.get_parameter('arrival_angular_speed_tolerance').value
        )

        detail = (
            f'位置误差={position_error:.3f}m, '
            f'朝向误差={math.degrees(yaw_error):.1f}°, '
            f'线速度={linear_speed:.3f}m/s, '
            f'角速度={angular_speed:.3f}rad/s'
        )
        verified = (
            position_error <= position_tolerance
            and yaw_error <= yaw_tolerance
            and linear_speed <= linear_tolerance
            and angular_speed <= angular_tolerance
        )
        return verified, detail

    def _verify_arrival(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
    ) -> Tuple[bool, str]:
        timeout = float(self.get_parameter('arrival_verify_timeout').value)
        stable_duration = float(
            self.get_parameter('arrival_stable_duration').value
        )
        deadline = time.monotonic() + timeout
        stable_since: Optional[float] = None
        last_detail = '尚无状态数据'

        while rclpy.ok() and time.monotonic() < deadline:
            verified, last_detail = self._arrival_state(
                target_x, target_y, target_yaw,
            )
            now = time.monotonic()
            if verified:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= stable_duration:
                    return True, last_detail
            else:
                stable_since = None
            time.sleep(0.10)

        return False, last_detail

    def _handle_cancel_navigation(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """请求取消当前导航。

        success=True 只表示：
        1. 当前确实没有活动导航；或者
        2. Nav2 已明确接受取消请求。

        它不表示机器人已经完成停车，最终终态由 /goto_place 响应确定。
        """
        _ = request

        # 当前完全没有导航，幂等成功。
        if (
            not self._navigation_in_progress.is_set()
            and not self._nav_client.has_active_goal()
        ):
            response.success = True
            response.message = '当前没有正在进行的导航'
            self.get_logger().info(
                f'/cancel_navigation: {response.message}'
            )
            return response

        # goto() 可能已经开始，但 Nav2 goal handle 尚未返回。
        # 在 cancel timeout 内短暂等待 goal handle 出现。
        timeout = float(
            self.get_parameter('nav2_cancel_timeout').value
        )
        deadline = time.monotonic() + timeout
        last_message = '尚未取得活动导航目标'

        while rclpy.ok() and time.monotonic() < deadline:
            accepted, message = self._nav_client.cancel_active_goal()
            last_message = message

            if accepted:
                response.success = True
                response.message = message
                self.get_logger().info(
                    f'/cancel_navigation: {message}'
                )
                return response

            # 如果导航在等待期间自然结束，现在已经无需取消。
            if (
                not self._navigation_in_progress.is_set()
                and not self._nav_client.has_active_goal()
            ):
                response.success = True
                response.message = '导航已经结束，无需取消'
                self.get_logger().info(
                    f'/cancel_navigation: {response.message}'
                )
                return response

            # 这些是明确失败，不需要继续重试。
            if any(
                marker in message
                for marker in ('发送失败', '读取', '拒绝', '超时', '空的取消响应')
            ):
                break

            # 最常见的可重试情况：goto 已开始，但 goal handle 尚未创建。
            time.sleep(0.05)

        response.success = False
        response.message = f'未能取消当前导航：{last_message}'
        self.get_logger().error(
            f'/cancel_navigation: {response.message}'
        )
        return response

    def _handle_goto_place(
        self,
        request: GotoPlace.Request,
        response: GotoPlace.Response,
    ) -> GotoPlace.Response:
        name = request.name.strip()
        if not name:
            response.success = False
            response.message = '地点名称不能为空'
            return response

        # PlaceStore 会在 get() 时检查文件签名并自动热重载。
        try:
            coordinates = self._store.get(name)
        except RuntimeError as exc:
            response.success = False
            response.message = f'读取地点文件失败: {exc}'
            return response

        if coordinates is None:
            response.success = False
            response.message = f'地点 "{name}" 不存在'
            return response

        x = coordinates['x']
        y = coordinates['y']
        yaw = coordinates['yaw']
        self.get_logger().info(
            f'开始导航到 "{name}": ({x:.3f}, {y:.3f}, {yaw:.3f})'
        )

        # 标记整个 NavClient.goto() 执行区间。
        # 取消服务依靠此状态处理 goal handle 尚未创建的短暂竞态。
        self._navigation_in_progress.set()
        try:
            nav_success, nav_message = self._nav_client.goto(x, y, yaw)
        finally:
            self._navigation_in_progress.clear()

        if not nav_success:
            response.success = False
            response.message = f'地点 "{name}" 导航失败：{nav_message}'
            return response

        verified, verification_message = self._verify_arrival(x, y, yaw)
        if not verified:
            response.success = False
            response.message = (
                f'Nav2 已成功，但地点 "{name}" 到达验证失败：'
                f'{verification_message}；{nav_message}'
            )
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'已到达地点 "{name}"；{verification_message}；{nav_message}'
        )
        self.get_logger().info(response.message)
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GotoPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
