"""命名地点管理节点。"""

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from rclpy.node import Node

from place_manager.config import (
    AMCL_POSE_MAX_AGE,
    AMCL_POSE_TOPIC,
    DEFAULT_PLACES_FILE,
    DELETE_PLACE_SERVICE,
    GET_PLACE_SERVICE,
    LIST_PLACES_SERVICE,
    MAP_FRAME,
    PLACE_MANAGER_NODE_NAME,
    SAVE_PLACE_SERVICE,
)
from place_manager.place_store import PlaceStore
from place_manager.pose_utils import quaternion_to_yaw
from place_manager.srv import DeletePlace, GetPlace, ListPlaces, SavePlace


class PlaceManagerNode(Node):
    """保存、删除、列举和查询命名地点。"""

    def __init__(self, places_file: str = DEFAULT_PLACES_FILE):
        super().__init__(PLACE_MANAGER_NODE_NAME)
        self.declare_parameter('places_file', places_file)
        self.declare_parameter('amcl_pose_max_age', AMCL_POSE_MAX_AGE)

        self._store = PlaceStore(
            str(self.get_parameter('places_file').value)
        )
        self._latest_pose: Optional[PoseWithCovarianceStamped] = None
        self._latest_pose_received_at: Optional[float] = None

        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            AMCL_POSE_TOPIC,
            self._pose_callback,
            10,
        )
        self._srv_save = self.create_service(
            SavePlace, SAVE_PLACE_SERVICE, self._handle_save_place,
        )
        self._srv_delete = self.create_service(
            DeletePlace, DELETE_PLACE_SERVICE, self._handle_delete_place,
        )
        self._srv_list = self.create_service(
            ListPlaces, LIST_PLACES_SERVICE, self._handle_list_places,
        )
        self._srv_get = self.create_service(
            GetPlace, GET_PLACE_SERVICE, self._handle_get_place,
        )

        self.get_logger().info(
            f'PlaceManagerNode 启动: {self._store.file_path} '
            f'（{len(self._store.list_names())} 个地点）'
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_pose = msg
        self._latest_pose_received_at = time.monotonic()

    def _handle_save_place(
        self,
        request: SavePlace.Request,
        response: SavePlace.Response,
    ) -> SavePlace.Response:
        name = request.name.strip()
        if not name:
            response.success = False
            response.message = '地点名称不能为空'
            return response
        if self._latest_pose is None or self._latest_pose_received_at is None:
            response.success = False
            response.message = '尚未收到 /amcl_pose，无法保存当前位置'
            return response
        if self._latest_pose.header.frame_id not in ('', MAP_FRAME):
            response.success = False
            response.message = (
                f'/amcl_pose frame_id={self._latest_pose.header.frame_id}，'
                f'期望 {MAP_FRAME}'
            )
            return response

        age = time.monotonic() - self._latest_pose_received_at
        max_age = float(self.get_parameter('amcl_pose_max_age').value)
        if age > max_age:
            response.success = False
            response.message = f'/amcl_pose 已过期（{age:.2f}s）'
            return response

        pose = self._latest_pose.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        try:
            self._store.add(name, pose.position.x, pose.position.y, yaw)
        except (RuntimeError, ValueError) as exc:
            response.success = False
            response.message = f'保存地点失败: {exc}'
            return response

        response.success = True
        response.message = f'已保存地点 "{name}"'
        self.get_logger().info(
            f'{response.message}: '
            f'({pose.position.x:.3f}, {pose.position.y:.3f}, {yaw:.3f})'
        )
        return response

    def _handle_delete_place(
        self,
        request: DeletePlace.Request,
        response: DeletePlace.Response,
    ) -> DeletePlace.Response:
        name = request.name.strip()
        try:
            deleted = self._store.delete(name)
        except RuntimeError as exc:
            response.success = False
            response.message = f'删除地点失败: {exc}'
            return response
        response.success = deleted
        response.message = (
            f'已删除地点 "{name}"' if deleted else f'地点 "{name}" 不存在'
        )
        return response

    def _handle_list_places(
        self,
        _request: ListPlaces.Request,
        response: ListPlaces.Response,
    ) -> ListPlaces.Response:
        try:
            response.names = self._store.list_names()
        except RuntimeError as exc:
            self.get_logger().error(f'列出地点失败: {exc}')
            response.names = []
        return response

    def _handle_get_place(
        self,
        request: GetPlace.Request,
        response: GetPlace.Response,
    ) -> GetPlace.Response:
        name = request.name.strip()
        try:
            data = self._store.get(name)
        except RuntimeError as exc:
            response.success = False
            response.message = f'读取地点失败: {exc}'
            response.pose = Pose2D()
            return response

        if data is None:
            response.success = False
            response.message = f'地点 "{name}" 不存在'
            response.pose = Pose2D()
            return response

        response.success = True
        response.message = f'已找到地点 "{name}"'
        response.pose = Pose2D(
            x=data['x'],
            y=data['y'],
            theta=data['yaw'],
        )
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PlaceManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
