#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from vision_interfaces.msg import DetectionResult
from ultralytics import YOLO
from kinematics_msgs.srv import GetRobotPose


def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1 - 2*(yy+zz), 2*(xy-wz),     2*(xz+wy)],
        [2*(xy+wz),     1 - 2*(xx+zz), 2*(yz-wx)],
        [2*(xz-wy),     2*(yz+wx),     1 - 2*(xx+yy)],
    ], dtype=np.float64)

def _pose_to_T(position, orientation) -> np.ndarray:
    R = _quat_to_rot(orientation.x, orientation.y, orientation.z, orientation.w)
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3, 3] = [position.x, position.y, position.z]
    return T


class YoloToWorldNode(Node):
    """
    YOLO + 深度 → camera坐标 → hand → base
    发布：
      - /vision_target (DetectionResult)
      - /world_model/objects (std_msgs/String, JSON)
    位姿获取采用“后台定时器异步刷新”，不会在任意回调里阻塞。
    """
    def __init__(self):
        super().__init__('simple_yolo_node')

        # ---------- 参数 ----------
        self.declare_parameter('rgb_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')          # Orbbec 16UC1(mm)
        self.declare_parameter('camera_info_topic', '/depth_cam/depth/camera_info')  # 与 depth 对应的 K
        self.declare_parameter('yolo_model', '/home/sundasheng/ros2_ws/src/vision_yolo/models/yolov8x.pt')
        self.declare_parameter('min_conf', 0.4)
        self.declare_parameter('publish_interval', 0.3)      # s
        self.declare_parameter('depth_in_meters', False)      # 16UC1=毫米 -> False
        self.declare_parameter('median_kernel', 5)            # 深度中值滤波核(奇数)
        self.declare_parameter('world_frame', 'base')
        self.declare_parameter('publish_if_no_pose', True)    # 没位姿时仅发 vision_target
        self.declare_parameter('pose_valid_secs', 2.0)        # 最近多少秒内的位姿算“新鲜”
        self.declare_parameter('pose_query_hz', 5.0)          # 异步查询频率
        self.declare_parameter('show_window', True)           # GUI 可关
        self.declare_parameter(
            'hand2cam_matrix',
            [ 0.0,  0.0,  1.0, -0.101,
             -1.0,  0.0,  0.0,  0.011,
              0.0, -1.0,  0.0,  0.045,
              0.0,  0.0,  0.0,  1.0 ]
        )

        # 读取参数
        p = self.get_parameter
        self.rgb_topic          = p('rgb_topic').value
        self.depth_topic        = p('depth_topic').value
        self.camera_info_topic  = p('camera_info_topic').value
        self.model_path         = p('yolo_model').value
        self.min_conf           = float(p('min_conf').value)
        self.publish_interval   = float(p('publish_interval').value)
        self.depth_in_meters    = bool(p('depth_in_meters').value)
        self.median_kernel      = int(p('median_kernel').value)
        self.world_frame        = str(p('world_frame').value)
        self.publish_if_no_pose = bool(p('publish_if_no_pose').value)
        self.pose_valid_secs    = float(p('pose_valid_secs').value)
        self.pose_query_hz      = float(p('pose_query_hz').value)
        self.show_window        = bool(p('show_window').value)

        try:
            self.hand2cam = np.array(p('hand2cam_matrix').value, dtype=np.float64).reshape(4,4)
        except Exception as e:
            self.get_logger().warn(f'hand2cam_matrix 非 4x4，回退单位阵: {e}')
            self.hand2cam = np.eye(4, dtype=np.float64)

        # ---------- 组件 ----------
        self.bridge = CvBridge()
        self.model  = YOLO(self.model_path)
        self.get_logger().info(f'✅ YOLOv8 已加载: {self.model_path}')

        # 订阅
        self.create_subscription(Image,      self.rgb_topic,         self.on_rgb,      10)
        self.create_subscription(Image,      self.depth_topic,       self.on_depth,    10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_caminfo,  10)

        # 发布
        self.pub_det = self.create_publisher(DetectionResult, '/vision_target',       10)
        self.pub_wm  = self.create_publisher(String,           '/world_model/objects', 10)

        # 末端位姿客户端 + 后台刷新
        self.pose_cli = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        if not self.pose_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('⚠️ /kinematics/get_current_pose 暂不可用；先只发像素+深度')

        # 位姿缓存
        self._last_T_hand  = None
        self._last_T_stamp = 0.0
        self._pose_req_outstanding = False  # 防止并发请求

        # 5Hz 定时刷新位姿（不会阻塞回调）
        if self.pose_query_hz > 0:
            self.create_timer(1.0 / self.pose_query_hz, self._refresh_hand_pose)

        # 缓存
        self.depth   = None      # np.float32, m
        self.K       = None      # 3x3
        self.last_ts = 0.0
        self._warned_no_pose = False

    # ---------- 后台刷新 hand pose（非阻塞） ----------
    def _refresh_hand_pose(self):
        if self._pose_req_outstanding or self.pose_cli is None:
            return
        try:
            self._pose_req_outstanding = True
            fut = self.pose_cli.call_async(GetRobotPose.Request())
            fut.add_done_callback(self._on_pose_result)
        except Exception as e:
            self._pose_req_outstanding = False
            self.get_logger().warn(f'pose req error: {e}')

    def _on_pose_result(self, fut):
        self._pose_req_outstanding = False
        try:
            res = fut.result()
            if res and res.success:
                T = _pose_to_T(res.pose.position, res.pose.orientation)
                self._last_T_hand  = T
                self._last_T_stamp = time.time()
        except Exception:
            pass

    # ---------- 回调 ----------
    def on_caminfo(self, msg: CameraInfo):
        try:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        except Exception as e:
            self.get_logger().error(f'CameraInfo 解析失败: {e}')

    def on_depth(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
            self.depth = (img if self.depth_in_meters else img/1000.0)  # mm→m
        except Exception as e:
            self.get_logger().error(f'Depth 解析失败: {e}')

    def on_rgb(self, msg: Image):
        now = time.time()
        if (now - self.last_ts) < self.publish_interval or self.K is None or self.depth is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB 解析失败: {e}')
            return

        # 1) YOLO
        res = self.model(frame, verbose=False)[0]

        # 2) 判断是否有“新鲜”的 hand 位姿
        have_world = (self._last_T_hand is not None) and ((now - self._last_T_stamp) < self.pose_valid_secs)
        if not have_world and not self._warned_no_pose:
            self.get_logger().warn('未拿到 hand 位姿或已过期；/world_model/objects 将跳过（可先用 publish_if_no_pose=true 只发 vision_target 进行对拍调试）')
            self._warned_no_pose = True

        # 3) 构造消息
        det = DetectionResult()
        det.image_width  = [frame.shape[1]]
        det.image_height = [frame.shape[0]]

        objects = []
        unix_ts = int(now)

        for b in res.boxes:
            cls_id  = int(b.cls[0])
            cls     = str(self.model.names[cls_id])
            conf    = float(b.conf[0])
            if conf < self.min_conf:
                continue

            x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
            u = (x1 + x2)//2
            v = (y1 + y2)//2

            det.class_name.append(cls)
            det.confidence.append(conf)
            det.center_x.append(int(u))
            det.center_y.append(int(v))

            z = self._depth_at(u, v)
            det.center_z.append(z if z is not None else -1.0)

            if have_world and z and z > 0:
                # 像素→camera
                fx, fy = self.K[0,0], self.K[1,1]
                cx, cy = self.K[0,2], self.K[1,2]
                x_cam  = (u - cx) / fx * z
                y_cam  = (v - cy) / fy * z
                p_cam  = np.array([x_cam, y_cam, z, 1.0], dtype=np.float64).reshape(4,1)

                # camera→base: base_T_cam = base_T_hand @ hand2cam
                base_T_cam = self._last_T_hand @ self.hand2cam
                p_base = base_T_cam @ p_cam
                X, Y, Z, _ = p_base.flatten().tolist()

                objects.append({
                    "id": len(objects)+1,
                    "class_name": cls,
                    "color": None,
                    "pose": {
                        "frame": self.world_frame,
                        "xyz": [round(X,3), round(Y,3), round(Z,3)],
                        "rpy": [0.0, 0.0, 1.57]
                    },
                    "confidence": round(conf,3),
                    "updated_at": unix_ts
                })

        # 4) 发布
        self.pub_det.publish(det)
        if have_world and objects:
            payload = json.dumps({"objects": objects}, ensure_ascii=False)
            self.pub_wm.publish(String(data=payload))
            o = objects[0]
            self.get_logger().info(f"🌍 {o['class_name']} → {o['pose']['xyz']} @ {self.world_frame}, conf={o['confidence']}")

        # 5) 叠字显示（可关闭）
        if self.show_window:
            try:
                for i, cls in enumerate(det.class_name):
                    cv2.circle(frame, (det.center_x[i], det.center_y[i]), 4, (0,255,0), -1)
                    cv2.putText(frame, f"{cls} {det.confidence[i]:.2f}",
                                (max(det.center_x[i]-40, 5), max(det.center_y[i]-10, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                cv2.imshow("YOLO", frame); cv2.waitKey(1)
            except Exception:
                pass

        self.last_ts = now

    # ---------- 工具 ----------
    def _depth_at(self, u, v):
        if self.depth is None: return None
        H, W = self.depth.shape[:2]
        if not (0 <= u < W and 0 <= v < H): return None

        if self.median_kernel >= 3 and self.median_kernel % 2 == 1:
            k = self.median_kernel
            x1, x2 = max(0, u-k//2), min(W, u+k//2+1)
            y1, y2 = max(0, v-k//2), min(H, v+k//2+1)
            patch  = self.depth[y1:y2, x1:x2]
            val    = patch[np.isfinite(patch) & (patch > 0.0)]
            return float(np.median(val)) if val.size else None
        else:
            d = float(self.depth[v, u])
            return d if (math.isfinite(d) and d > 0.0) else None


def main(args=None):
    rclpy.init(args=args)
    node = YoloToWorldNode()
    try:
        rclpy.spin(node)  # 单线程就足够，因不再阻塞
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()

