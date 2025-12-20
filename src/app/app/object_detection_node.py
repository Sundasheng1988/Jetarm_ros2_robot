import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
import apriltag
from kinematics_msgs.srv import GetRobotPose


class ObjectPosePublisher(Node):
    def __init__(self):
        super().__init__('object_pose_publisher')

        self.bridge = CvBridge()
        self.latest_camera_info = None  # 缓存 camera_info

        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 10)

        self.publisher = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.detector = apriltag.Detector()
        self.tag_size = 0.025  # 米

        self.pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        if not self.pose_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("❌ 服务 /kinematics/get_current_pose 未响应")
        else:
            self.get_logger().info("✅ 成功连接服务 /kinematics/get_current_pose")

        self.hand2cam = np.array([
            [0.0, 0.0, 1.0, -0.101],
            [-1.0, 0.0, 0.0, 0.011],
            [0.0, -1.0, 0.0, 0.045],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def image_callback(self, img_msg):
        if self.latest_camera_info is None:
            self.get_logger().warn("⏳ 等待 camera_info 数据...")
            return

        self.get_logger().info("📸 收到图像帧，开始检测")

        info_msg = self.latest_camera_info
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        camera_matrix = np.array(info_msg.k).reshape(3, 3)
        dist_coeffs = np.array(info_msg.d)

        detections = self.detector.detect(cv_image)
        if not detections:
            self.get_logger().info("🔍 未检测到 AprilTag")
            return

        tag = detections[0]
        object_points = np.array([
            [-self.tag_size / 2, -self.tag_size / 2, 0],
            [ self.tag_size / 2, -self.tag_size / 2, 0],
            [ self.tag_size / 2,  self.tag_size / 2, 0],
            [-self.tag_size / 2,  self.tag_size / 2, 0],
        ])
        image_points = np.array(tag.corners, dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        if not success:
            self.get_logger().error("❌ solvePnP 失败")
            return

        R, _ = cv2.Rodrigues(rvec)
        T_cam = np.eye(4)
        T_cam[:3, :3] = R
        T_cam[:3, 3] = tvec.flatten()

        request = GetRobotPose.Request()
        future = self.pose_client.call_async(request)
        future.add_done_callback(lambda fut: self.handle_pose_response(fut, T_cam))

    def handle_pose_response(self, future, target_in_cam):
        result = future.result()
        if result is None:
            self.get_logger().error("❌ 服务调用失败，future 无返回")
            return

        pose = result.pose
        t = pose.position
        q = pose.orientation

        hand2base = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        hand2base[:3, 3] = [t.x, t.y, t.z]

        base_T_obj = hand2base @ self.hand2cam @ target_in_cam
        obj_pos = base_T_obj[:3, 3]

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = obj_pos[0]
        pose_msg.pose.position.y = obj_pos[1]
        pose_msg.pose.position.z = obj_pos[2]
        pose_msg.pose.orientation.w = 1.0

        self.publisher.publish(pose_msg)

        self.get_logger().info(
            f"📦 发布目标 3D 位姿: x={obj_pos[0]:.3f}, y={obj_pos[1]:.3f}, z={obj_pos[2]:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

