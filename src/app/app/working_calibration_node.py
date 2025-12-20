import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag
import yaml
from kinematics_msgs.srv import GetRobotPose
import tf_transformations


class WorkingCalibrationNode(Node):
    def __init__(self):
        super().__init__('working_calibration_node')
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None

        # 加载transform.yaml中的转换矩阵
        config_path = '/home/sundasheng/ros2_ws/src/working_app/config/transform.yaml'
        self.load_transform_yaml(config_path)

        # 加载相机内参
        camera_info_path = '/home/sundasheng/ros2_ws/src/working_app/config/camera_info.yaml'
        self.load_camera_info(camera_info_path)

        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_callback, 10)

        self.detector = apriltag.Detector()
        self.get_logger().info('✅ working_calibration_node 启动，等待相机数据...')

    def load_transform_yaml(self, path):
        # 读取transform.yaml文件
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            self.hand2cam = np.array(data['hand2cam_transformation_matrix'])  # hand2cam 固定
            self.world_pose = np.array(data['white_area_pose_world'])  # extrinsic 外参从transform.yaml读取

    def load_camera_info(self, path):
        # 读取相机内参
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            K = data['camera_matrix']['data']
            self.K = np.array(K).reshape(3,3)
            self.fx = self.K[0,0]
            self.fy = self.K[1,1]
            self.cx = self.K[0,2]
            self.cy = self.K[1,2]

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def image_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            self.get_logger().warning('未收到CameraInfo或Depth，跳过本帧')
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)

        for tag in tags:
            u, v = int(tag.center[0]), int(tag.center[1])
            if u < 0 or u >= self.depth_image.shape[1] or v < 0 or v >= self.depth_image.shape[0]:
                continue

            z_raw = float(self.depth_image[v, u])
            z = z_raw / 1000.0  # ⚠️ 如单位为毫米，需 z = z_raw / 1000.0

            if z < 0.05 or z > 5.0:
                self.get_logger().warning(f"⚠️ Tag {tag.tag_id} 深度异常 z={z:.3f}")
                continue

            # 计算相机坐标系中的 (x, y, z)
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            cam_point = np.array([x, y, z, 1.0]).reshape(4,1)

            # 使用 hand2cam 外参将相机坐标转换为手坐标
            hand_point = self.hand2cam @ cam_point

            # 获取当前末端位置
            self.get_hand2base()

            # 使用 world_pose 将手坐标转换为世界坐标
            world_point = self.world_pose @ hand_point
            xyz = world_point[:3].flatten()

            # 打印调试信息
            self.get_logger().info(
                f"\n📍 TagID={tag.tag_id}\n"
                f"   ▶️ 像素坐标: u={u}, v={v}\n"
                f"   ▶️ 深度值 z={z:.6f} m （原始值 z_raw={z_raw:.2f}）\n"
                f"   ▶️ 相机坐标: x={x:.6f}, y={y:.6f}, z={z:.6f}\n"
                f"   ▶️ Hand坐标: [{', '.join(f'{v[0]:.6f}' for v in hand_point[:3])}]\n"
                f"   ▶️ 世界坐标: [{', '.join(f'{v:.6f}' for v in xyz)}]"
            )

            # 可视化
            cv2.circle(frame, (u, v), 5, (0,255,0), -1)
            cv2.putText(frame, f"id:{tag.tag_id}", (u+10, v-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        cv2.imshow("Working Calibration", frame)
        cv2.waitKey(1)

    def get_hand2base(self):
        # 获取当前机械臂末端位姿
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
        
            # 计算 hand2base（末端到基座的变换矩阵）
            hand2base = tf_transformations.quaternion_matrix([pose_r.x, pose_r.y, pose_r.z, pose_r.w])
            hand2base[:3, 3] = [pose_t.x, pose_t.y, pose_t.z]
            self.get_logger().info(f"📍 hand2base (末端到基座的变换矩阵):\n{np.round(hand2base, 4)}")
            
            return hand2base

        future.add_done_callback(handle_response)


def main(args=None):
    rclpy.init(args=args)
    node = WorkingCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

