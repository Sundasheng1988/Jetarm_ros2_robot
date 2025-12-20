import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
import message_filters
import apriltag  # ✅ 关键替代
from kinematics_msgs.srv import GetRobotPose
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor  # 顶部加这一行
import time

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/rgb/camera_info')

        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.info_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)

        self.get_logger().info("✅ Calibration node started")

        self.at_detector = apriltag.Detector()  # ✅ 使用 apriltag 库初始化
        self.frames_collected = []
        self.max_frames = 5
        self.tag_size = 0.025  # 米

        # 固定的 hand2cam 外参矩阵 (手眼标定的转换矩阵)
        self.hand2cam_transformation_matrix = np.array([
            [0.0, 0.0, 1.0, -0.101],
            [-1.0, 0.0, 0.0, 0.011],
            [0.0, -1.0, 0.0, 0.045],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def image_callback(self, img_msg, info_msg):
        if len(self.frames_collected) >= self.max_frames:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        camera_matrix = np.array(info_msg.k).reshape(3, 3)
        dist_coeffs = np.array(info_msg.d)

        detections = self.at_detector.detect(cv_image)
        if len(detections) == 0:
            self.get_logger().info("🎯 当前帧未检测到 Tag")
            return

        tag = detections[0]
        object_points = np.array([
            [-self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2,  self.tag_size/2, 0],
            [-self.tag_size/2,  self.tag_size/2, 0],
        ])
        image_points = np.array(tag.corners, dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        if not success:
            self.get_logger().error("❌ solvePnP失败")
            return

        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()

        self.frames_collected.append(T)
        self.get_logger().info(f"📍 已采集第 {len(self.frames_collected)} 帧 Tag Pose")

        if len(self.frames_collected) == self.max_frames:
            self.get_logger().info("⏳ 已采集所有帧，准备计算外参矩阵")
            self.timer = self.create_timer(0.01, self.compute_average_extrinsic, callback_group=None)

    def compute_average_extrinsic(self):
        self.timer.cancel()  # 自动注销
        if hasattr(self, 'already_computing') and self.already_computing:
            return
        self.already_computing = True  # 防止多次触发
        
        avg_pose = np.mean(np.array(self.frames_collected), axis=0)
        self.get_logger().info("✅ 完成平均外参矩阵计算")
        print("\n📌 avg_tag_pose_cam_frame (4x4):\n", np.round(avg_pose, 4))

        # 固定的 hand2cam 外参（手眼标定的转换矩阵）
        hand2cam = self.hand2cam_transformation_matrix

        # 获取真实的 hand2base（末端在 base 坐标系下的位姿）
        client = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("⛔️ 服务 /kinematics/get_current_pose 未启动")
            return

        request = GetRobotPose.Request()
        future = client.call_async(request)

        def handle_response(fut):
            if fut.result() is None:
                self.get_logger().error("❌ 服务调用失败，future 无返回")
                return

            pose_t = fut.result().pose.position
            pose_r = fut.result().pose.orientation
        
            hand2base = tf_transformations.quaternion_matrix([pose_r.x, pose_r.y, pose_r.z, pose_r.w])
            hand2base[:3, 3] = [pose_t.x, pose_t.y, pose_t.z]
            print("\n📌 hand2base (from service):\n", np.round(hand2base, 4))
            
            # 提取末端世界坐标
            x, y, z = pose_t.x, pose_t.y, pose_t.z
            qx, qy, qz, qw = pose_r.x, pose_r.y, pose_r.z, pose_r.w

            self.get_logger().info(f"🦾 机械臂末端世界坐标：")
            self.get_logger().info(f"📍 位置：x={x:.4f}, y={y:.4f}, z={z:.4f}")
            self.get_logger().info(f"📐 姿态四元数：qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}")

            base2tag = hand2base @ hand2cam @ avg_pose
            tag_pos_in_base = base2tag[:3, 3]
            
            self.get_logger().info(f"📌 Tag 在 base 坐标系中的位置（单位：米）: x={tag_pos_in_base[0]:.4f}, y={tag_pos_in_base[1]:.4f}, z={tag_pos_in_base[2]:.4f}")
            print("\n📌 base2tag (估算):\n", np.round(base2tag, 4))
            self.get_logger().info("🎉 标定流程完成，结果已打印")

        future.add_done_callback(handle_response)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()

