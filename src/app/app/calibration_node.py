#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import time
import yaml
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from dt_apriltags import Detector
from sensor_msgs.msg import Image, CameraInfo
from kinematics_msgs.srv import GetRobotPose
from rclpy.qos import qos_profile_sensor_data


# ------------------ minimal math helpers ------------------
def quat_wxyz_to_R(w, x, y, z):
    n = np.sqrt(w*w + x*x + y*y + z*z) + 1e-12
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),    2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),    2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),    1-2*(x*x+y*y)]
    ], dtype=np.float64)

def xyz_quat_to_mat(xyz, wxyz):
    w, x, y, z = wxyz
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_wxyz_to_R(w, x, y, z)
    T[:3, 3] = np.array(xyz, dtype=np.float64)
    return T

def R_to_euler_xyz_deg(R):
    R = np.asarray(R, dtype=np.float64)
    sy = np.clip(-R[2,0], -1.0, 1.0)
    x = np.arctan2(R[2,1], R[2,2])
    y = np.arcsin(sy)
    z = np.arctan2(R[1,0], R[0,0])
    return np.degrees([x, y, z]).tolist()

def mat_to_xyz_euler(T):
    return T[:3, 3].tolist(), R_to_euler_xyz_deg(T[:3, :3])

def xyz_euler_to_mat(xyz, euler_deg):
    rx, ry, rz = np.radians(euler_deg)
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
    Ry = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
    Rz = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
    R = Rx @ Ry @ Rz
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3, 3] = np.array(xyz, dtype=np.float64)
    return T

def extristric_plane_shift(tvec, rmat, dz):
    T = np.eye(4)
    T[:3,:3] = rmat
    T[:3, 3] = np.array(tvec).reshape(3)
    shift = np.eye(4); shift[2,3] = dz
    T2 = T @ shift
    return T2[:3, 3].reshape(3,1), T2[:3,:3]

def draw_tags(img, tags):
    out = img.copy()
    for t in tags:
        for (x, y) in np.int32(t.corners):
            cv2.circle(out, (x, y), 2, (0, 255, 0), -1)
        c0 = tuple(np.int32(t.corners[0]))
        cv2.putText(out, f"id:{t.tag_id}", c0, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
    return out
# -----------------------------------------------------------


class CalibrationNode(Node):
    # 手到相机（手->相机）固定外参
    hand2cam_tf_matrix = np.array([
        [0.0,  0.0,  1.0, -0.101],
        [-1.0, 0.0,  0.0,  0.011],
        [0.0, -1.0,  0.0,  0.045],
        [0.0,  0.0,  0.0,  1.0]
    ], dtype=np.float64)

    def __init__(self, name):
        super().__init__(name)

        # 采样配置
        self.tag_size        = 0.025
        self.allowed_tag_ids = set([100])   # 你的场景 id=100
        self.min_samples     = 5
        self.max_timeout_s   = 10.0         # ← 10 秒

        # 状态
        self.camera_type = os.getenv('CAMERA_TYPE', 'GEMINI')
        self.calibration_step = 0           # 0-空闲, 1-采样, 2-采满等待计算, 20-完成
        self.tags = []                      # 采样帧（dict）
        self.K = None
        self.D = None
        self.lock = threading.RLock()
        self.running = False
        self.thread = None
        self.err_msg = None
        self.imgpts = None

        # 同步
        self.samples_ready = threading.Event()

        # 配置/持久化
        self.config_file = 'transform.yaml'
        self.config_path = "/home/sundasheng/ros2_ws/src/app/config/"
        os.makedirs(self.config_path, exist_ok=True)

        # 白区（尺寸米）
        self.white_area_width = 0.175
        self.white_area_height = 0.135

        # I/O
        self.bridge = CvBridge()
        self.result_image_pub = self.create_publisher(Image, '~/image_result', 10)

        # 订阅与服务
        self.image_sub = None
        self.camera_info_sub = None
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(Trigger, '~/start', self.start_calibration_srv_callback)

        # AprilTag
        self.at_detector = Detector(
            searchpath=['apriltags'],
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # 存档
        self.extristric = None
        self.white_area_pose_cam = np.eye(4, dtype=np.float64)
        self.white_area_pose_world = np.eye(4, dtype=np.float64)
        cfg_path = os.path.join(self.config_path, self.config_file)
        if os.path.exists(cfg_path):
            try:
                with open(cfg_path, 'r', encoding='utf-8') as f:
                    cfg = yaml.safe_load(f) or {}
                if 'extristric' in cfg:
                    self.extristric = np.array(cfg['extristric'], dtype=np.float64)
                if 'white_area_pose_cam' in cfg:
                    self.white_area_pose_cam = np.array(cfg['white_area_pose_cam'], dtype=np.float64)
                if 'white_area_pose_world' in cfg:
                    self.white_area_pose_world = np.array(cfg['white_area_pose_world'], dtype=np.float64)
            except Exception as e:
                self.get_logger().warn(f"读取 {cfg_path} 失败：{e}")
        if self.extristric is None:
            self.extristric = np.vstack([np.zeros((1,3)), np.eye(3)])

        # 末端位姿（采样时不再等待服务，避免卡 executor）
        self.endpoint = np.eye(4, dtype=np.float64)
        self.get_current_pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        # 由主线程定时异步刷新（不等待）
        self.create_timer(0.5, self._refresh_endpoint_async)

        # 杂项
        self._first_frame_logged = False
        self._last_stat_t = 0.0

        self.get_logger().info("✅ calibration_node 启动（去 SDK 版）")

    # --------- 非阻塞异步刷新末端位姿（主线程定时器执行） ----------
    def _refresh_endpoint_async(self):
        try:
            if not self.get_current_pose_client.service_is_ready():
                return
            future = self.get_current_pose_client.call_async(GetRobotPose.Request())
            # 让回调线程去完成 future；我们这里不等待、不 spin。
            def _done(_fut):
                try:
                    res = _fut.result()
                    if res and res.success:
                        p = res.pose.position
                        q = res.pose.orientation
                        self.endpoint = xyz_quat_to_mat([p.x, p.y, p.z], [q.w, q.x, q.y, q.z])
                except Exception:
                    pass
            future.add_done_callback(_done)
        except Exception:
            pass

    # ---------------- 核心流程（后台线程，但绝不 spin） ----------------
    def calibration_proc(self):
        # 不再在这里等待 get_current_pose，直接用最近一次的 self.endpoint（或单位阵）。
        t0 = time.time()
        self.tags = []
        self.samples_ready.clear()
        self.calibration_step = 1
        self.get_logger().info(f"⏳ sampling started: need {self.min_samples} frames, timeout={self.max_timeout_s:.1f}s")

        while not self.samples_ready.is_set() and (time.time() - t0) < self.max_timeout_s:
            time.sleep(0.02)

        if not self.samples_ready.is_set():
            n = len(self.tags)
            self.get_logger().warn(f"⏰ sampling timeout: only collected {n}/{self.min_samples} frames")
            self.err_msg = "Time out, calibrate failed!!!"
            time.sleep(0.2)
            self.err_msg = None
            self.finish_proc()
            return

        self.get_logger().info(f"✅ sampling finished with {len(self.tags)} frames, proceed to PnP")

        # 计算识别区域中心位姿
        poses = [self.xyzR_to_T(t['pose_t'], t['pose_R']) for t in self.tags]
        vecs = np.array([p.ravel() for p in poses], dtype=np.float64)
        avg_pose = vecs.mean(axis=0).reshape(4, 4)
        pose_end = self.hand2cam_tf_matrix @ avg_pose
        pose_world = self.endpoint @ pose_end

        white_area_pose_cam = avg_pose.copy()
        white_area_pose_world = pose_world.copy()

        # PnP
        world_points = np.array(
            [(-self.tag_size/2, -self.tag_size/2, 0),
             ( self.tag_size/2, -self.tag_size/2, 0),
             ( self.tag_size/2,  self.tag_size/2, 0),
             (-self.tag_size/2,  self.tag_size/2, 0)] * len(self.tags),
            dtype=np.float64
        )
        image_points = np.vstack([t['corners'] for t in self.tags]).reshape((-1, 2))

        if self.K is None or self.D is None:
            self.err_msg = "No camera info received!"
            self.get_logger().error(self.err_msg)
            self.finish_proc()
            return

        ok, rvec, tvec = cv2.solvePnP(
            world_points, image_points,
            self.K, self.D,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            self.get_logger().warn("solvePnP 失败，使用单位外参")
            rmat = np.eye(3)
            tvec = np.zeros((3,1))
        else:
            rmat, _ = cv2.Rodrigues(rvec)

        extristric = np.vstack([tvec.reshape(1,3), rmat])

        # 保存
        self.extristric = extristric
        self.white_area_pose_cam = white_area_pose_cam
        self.white_area_pose_world = white_area_pose_world

        xyz_cam, e_cam = mat_to_xyz_euler(white_area_pose_cam)
        xyz_w, e_w = mat_to_xyz_euler(white_area_pose_world)
        self.get_logger().info(f"[cam]   xyz={np.round(xyz_cam,4)} euler={np.round(e_cam,1)}")
        self.get_logger().info(f"[world] xyz={np.round(xyz_w,4)} euler={np.round(e_w,1)}")

        # 写回 yaml
        try:
            self.update_yaml_data({
                'white_area_pose_cam': white_area_pose_cam.tolist(),
                'white_area_pose_world': white_area_pose_world.tolist(),
                'extristric': extristric.tolist(),
            }, os.path.join(self.config_path, self.config_file))
            self.get_logger().info("transform.yaml 已更新")
        except Exception as e:
            self.get_logger().warn(f"写 transform.yaml 失败：{e}")

        # 预览一次
        self.calibration_step = 20
        self.draw_rectangle_safe()
        time.sleep(0.1)
        self.finish_proc()

    def finish_proc(self):
        self.calibration_step = 0
        self.thread = None

    # ---------------- 工具 ----------------
    def xyzR_to_T(self, t, R_):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = np.array(R_, dtype=np.float64)
        T[:3, 3] = np.array(t, dtype=np.float64).reshape(3)
        return T

    def update_yaml_data(self, new_data, yaml_file):
        data = {}
        if os.path.exists(yaml_file):
            with open(yaml_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        data.update(new_data)
        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True)

    # ---------------- 服务回调 ----------------
    def start_calibration_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start calibration")
        with self.lock:
            if self.image_sub is None:
                response.success = False
                response.message = "Please call enter service first"
                return response

            # 准备新的采样
            self.samples_ready.clear()
            self.tags = []
            self.err_msg = None
            self.calibration_step = 1
            self.get_logger().info("✅ sampling gate opened (step=1)")

            if self.thread is None:
                self.thread = threading.Thread(target=self.calibration_proc, daemon=True)
                self.thread.start()
                response.success = True
                response.message = "start"
                return response
            else:
                response.success = False
                response.message = "Calibration..."
                return response

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "loading calibration")

        self.image_sub = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        cam_info_topic = '/camera_info' if self.camera_type == 'USB_CAM' \
                         else '/depth_cam/depth/camera_info'
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            cam_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        self.running = True
        response.success = True
        response.message = "start"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop calibration")
        self.running = False
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
            if self.camera_info_sub is not None:
                self.destroy_subscription(self.camera_info_sub)
                self.camera_info_sub = None
        except Exception as e:
            self.get_logger().error(str(e))
        response.success = True
        response.message = "stop"
        return response

    # ---------------- 相机回调与绘制 ----------------
    def camera_info_callback(self, msg):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
        self.K = np.matrix(new_K)
        self.D = np.zeros((5, 1), dtype=np.float64)  # 如需真实畸变，改为 D
        if not hasattr(self, "_caminfo_logged"):
            self.get_logger().info(
                f"📷 CameraInfo received. K[0,0]={float(self.K[0,0]):.1f}, "
                f"K[1,1]={float(self.K[1,1]):.1f}, cx={float(self.K[0,2]):.1f}, cy={float(self.K[1,2]):.1f}"
            )
            self._caminfo_logged = True

    def draw_rectangle_safe(self):
        try:
            white_area_center = self.white_area_pose_world.reshape(4, 4)
            white_area_cam = self.white_area_pose_cam.reshape(4, 4)

            lt = white_area_center @ xyz_euler_to_mat(( self.white_area_height/2,  self.white_area_width/2, 0.0), (0,0,0))
            lb = white_area_center @ xyz_euler_to_mat((-self.white_area_height/2,  self.white_area_width/2, 0.0), (0,0,0))
            rb = white_area_center @ xyz_euler_to_mat((-self.white_area_height/2, -self.white_area_width/2, 0.0), (0,0,0))
            rt = white_area_center @ xyz_euler_to_mat(( self.white_area_height/2, -self.white_area_width/2, 0.0), (0,0,0))

            corners_cam = np.stack([lt, lb, rb, rt, white_area_center], axis=0)
            corners_cam = np.linalg.inv(self.endpoint @ self.hand2cam_tf_matrix) @ corners_cam
            corners_cam = np.linalg.inv(white_area_cam) @ corners_cam
            corners_cam = corners_cam[:, :3, 3:].reshape((-1, 3))

            if self.K is None or self.D is None:
                return

            tvec = self.extristric[:1]   # (1,3)
            rmat = self.extristric[1:]   # (3,3)

            _, _ = cv2.projectPoints(corners_cam[-1:], rmat, tvec.reshape(3,1), self.K, self.D)
            tvec2, rmat2 = extristric_plane_shift(tvec.reshape(3,1), rmat, 0.030)
            _imgpts, _ = cv2.projectPoints(corners_cam[:-1], rmat2, tvec2, self.K, self.D)
            self.imgpts = np.int32(_imgpts).reshape(-1, 2)
        except Exception as e:
            self.get_logger().warn(f"draw_rectangle_safe 失败：{e}")
            self.imgpts = None

    def image_callback(self, ros_image):
        if not self.running:
            return

        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        if cv_image.ndim == 2:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        else:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        if not self._first_frame_logged:
            self.get_logger().info(f"📷 First image received: shape={rgb_image.shape}")
            self._first_frame_logged = True

        now = time.time()
        if now - self._last_stat_t > 1.0:
            self._last_stat_t = now
            self.get_logger().info(f"ℹ️ step={self.calibration_step}, collected={len(self.tags)}/{self.min_samples}")

        if self.K is None:
            return

        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        tags = self.at_detector.detect(
            gray, True,
            (float(self.K[0,0]), float(self.K[1,1]), float(self.K[0,2]), float(self.K[1,2])),
            self.tag_size
        )
        ids = [int(t.tag_id) for t in tags]
        self.get_logger().info(f"🔎 detect {len(tags)} tags, ids={ids}")

        # 采样：满足就追加
        if self.calibration_step == 1 and len(self.tags) < self.min_samples:
            if len(tags) == 1 and (not self.allowed_tag_ids or int(tags[0].tag_id) in self.allowed_tag_ids):
                t = tags[0]
                rec = {
                    'pose_t' : np.array(t.pose_t, dtype=np.float64).reshape(3),
                    'pose_R' : np.array(t.pose_R, dtype=np.float64).reshape(3,3),
                    'corners': np.array(t.corners, dtype=np.float64).reshape(4,2)
                }
                if not (np.any(np.isnan(rec['pose_t'])) or np.any(np.isnan(rec['pose_R']))):
                    self.tags.append(rec)
                    self.get_logger().info(f"✅ APPEND id={int(t.tag_id)} -> {len(self.tags)}/{self.min_samples}")
                    if len(self.tags) >= self.min_samples:
                        self.get_logger().info("🎉 SIGNAL samples_ready (>= min_samples)")
                        self.samples_ready.set()
                        self.calibration_step = 2
            else:
                self.err_msg = "Keep exactly ONE tag in view"

        # 采样阶段为了稳妥不发布图像；若要预览可取消注释：
        # vis = draw_tags(rgb_image, tags)
        # self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(vis, "rgb8"))


def main():
    rclpy.init()
    node = CalibrationNode('calibration')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

