#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, json
import numpy as np
import cv2, yaml
from pathlib import Path
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from vision_interfaces.msg import DetectionResult
from ultralytics import YOLO
from kinematics_msgs.srv import GetRobotPose


# ---------------------- math utils ----------------------
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


# ---------------------- node ----------------------
class AppCompatibleYoloNode(Node):
    def __init__(self):
        super().__init__('app_compatible_yolo_node')

        # ---- topics & model ----
        self.declare_parameter('rgb_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_cam/depth/camera_info')
        self.declare_parameter('yolo_model', '/home/sundasheng/ros2_ws/src/vision_yolo/models/yolov8x.pt')

        # ---- behavior ----
        self.declare_parameter('publish_interval', 0.30)
        self.declare_parameter('min_conf', 0.20)  # 放宽，先确保能出框
        self.declare_parameter('show_window', True)

        # ---- frames & hand2cam ----
        self.declare_parameter('world_frame', 'base')
        self.declare_parameter(
            'hand2cam_matrix',
            [ 0.0,  0.0,  1.0, -0.101,
             -1.0,  0.0,  0.0,  0.011,
              0.0, -1.0,  0.0,  0.045,
              0.0,  0.0,  0.0,  1.0 ]
        )
        self.declare_parameter('pose_valid_secs', 2.0)
        self.declare_parameter('pose_query_hz', 5.0)

        # ---- white area / calib ----
        self.declare_parameter('transform_yaml', '/home/sundasheng/ros2_ws/src/app/config/transform.yaml')
        self.declare_parameter('white_area_size', [0.175, 0.135])   # w,h (m)
        self.declare_parameter('mask_apriltag', True)               # 只遮挡白区中心小块（不整帧掩黑）

        # ---- colors (LAB from your file) ----
        self.declare_parameter('lab_config', '/home/sundasheng/ros2_ws/src/app/config/lab_config.yaml')
        self.declare_parameter('fallback_only_red', True)
        self.declare_parameter('min_area_px', 500)
        self.declare_parameter('max_area_px', 7000)

        # ---- YOLO class filter (post-filter) ----
        self.declare_parameter('use_class_whitelist', True)
        self.declare_parameter('class_whitelist',
            'sports ball,cup,bottle,cell phone,remote,mouse,keyboard,traffic light,box,bowl,clock')
        self.declare_parameter('class_blacklist', 'person')

        # ---- APP-like projection & compensation ----
        self.declare_parameter('pick_plane_z', 0.030)   # Z 固定抬高到 3cm
        self.declare_parameter('y_flip', True)          # APP: world_pose[1] = -world_pose[1]
        self.declare_parameter('offset', [0.0, 0.0, 0.0])
        self.declare_parameter('scale',  [1.0, 1.0, 1.0])
        self.declare_parameter('droop_k', 0.025)
        self.declare_parameter('droop_r0', 0.15)
        self.declare_parameter('droop_r1', 0.20)

        # ---- params ----
        g = self.get_parameter
        self.rgb_topic        = g('rgb_topic').value
        self.camera_info_topic= g('camera_info_topic').value
        self.model_path       = g('yolo_model').value
        self.publish_interval = float(g('publish_interval').value)
        self.min_conf         = float(g('min_conf').value)
        self.show_window      = bool(g('show_window').value)
        self.world_frame      = str(g('world_frame').value)
        self.transform_yaml   = str(g('transform_yaml').value)
        self.white_area_size  = [float(x) for x in g('white_area_size').value]
        self.mask_apriltag    = bool(g('mask_apriltag').value)
        self.lab_config_path  = str(g('lab_config').value)
        self.only_red         = bool(g('fallback_only_red').value)
        self.min_area_px      = int(g('min_area_px').value)
        self.max_area_px      = int(g('max_area_px').value)
        self.use_class_whitelist = bool(g('use_class_whitelist').value)
        self.class_whitelist  = str(g('class_whitelist').value or '')
        self.class_blacklist  = str(g('class_blacklist').value or '')
        self.pick_plane_z     = float(g('pick_plane_z').value)
        self.y_flip           = bool(g('y_flip').value)
        self.offset           = [float(x) for x in g('offset').value]
        self.scale            = [float(x) for x in g('scale').value]
        self.droop_k          = float(g('droop_k').value)
        self.droop_r0         = float(g('droop_r0').value)
        self.droop_r1         = float(g('droop_r1').value)
        try:
            self.hand2cam = np.array(g('hand2cam_matrix').value, dtype=np.float64).reshape(4,4)
        except Exception as e:
            self.get_logger().warn(f'hand2cam_matrix 非 4x4，回退单位阵: {e}')
            self.hand2cam = np.eye(4, dtype=np.float64)

        # ---- runtime objects ----
        self.bridge = CvBridge()
        self.model  = YOLO(self.model_path)
        self.get_logger().info(f'✅ YOLOv8 已加载: {self.model_path}')
        self.allowed_indices = self._compute_allowed_classes()

        self.create_subscription(Image, self.rgb_topic, self.on_rgb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_caminfo, 10)
        self.pub_det = self.create_publisher(DetectionResult, '/vision_target', 10)
        self.pub_wm  = self.create_publisher(String, '/world_model/objects', 10)

        self.pose_cli = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        if not self.pose_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('⚠️ /kinematics/get_current_pose 暂不可用')

        self._last_T_hand  = None
        self._last_T_stamp = 0.0
        self._pose_req_outstanding = False
        if float(g('pose_query_hz').value) > 0:
            self.create_timer(1.0/float(g('pose_query_hz').value), self._refresh_hand_pose)

        # 缓存
        self.K = None
        self.last_ts = 0.0
        self._roi_poly = None
        self._white_pose_world = None
        self._H_i2w = None

        self._load_white_pose()
        self.lab_ranges = self._load_lab_ranges()

    # -------------------- init helpers --------------------
    def _compute_allowed_classes(self):
        try:   names_map = dict(self.model.names)
        except Exception: names_map = {i:n for i,n in enumerate(self.model.names)}
        wl = {s.strip().lower() for s in self.class_whitelist.split(',') if s.strip()}
        bl = {s.strip().lower() for s in self.class_blacklist.split(',') if s.strip()}
        idx = []
        for i,nm in names_map.items():
            n = (nm or '').lower()
            if self.use_class_whitelist and wl:
                if n in wl: idx.append(i)
            else:
                if n not in bl: idx.append(i)
        return idx

    def _load_white_pose(self):
        p = Path(self.transform_yaml)
        if not p.exists(): return
        try:
            self._white_pose_world = np.array(
                yaml.safe_load(p.read_text())['white_area_pose_world'],
                dtype=np.float64
            ).reshape(4,4)
        except Exception as e:
            self.get_logger().warn(f'读取 transform.yaml 失败: {e}')

    def _load_lab_ranges(self):
        """兼容你上传的 lab_config.yaml 结构（/**/ros__parameters/color_range_list/...）"""
        rng = {}
        try:
            p = Path(self.lab_config_path)
            data = yaml.safe_load(p.read_text())
            cl = data['/**']['ros__parameters']['color_range_list']
            for name in cl:
                mn = tuple(cl[name]['min']); mx = tuple(cl[name]['max'])
                rng[name] = (mn, mx)  # LAB min/max
        except Exception as e:
            self.get_logger().warn(f'LAB 阈值加载失败，使用默认: {e}')
            rng['red']   = ((0,140,120),(255,195,200))
            rng['green'] = ((0, 50,120),(255,130,170))
            rng['blue']  = ((0,120, 50),(255,180,125))
        return rng

    # -------------------- pose refresh --------------------
    def _refresh_hand_pose(self):
        if self._pose_req_outstanding or self.pose_cli is None: return
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
                self._build_roi_and_homography()
        except Exception:
            pass

    # -------------------- subs --------------------
    def on_caminfo(self, msg: CameraInfo):
        try:
            K = np.array(msg.k, dtype=np.float64).reshape(3,3)
            D = np.array(msg.d, dtype=np.float64)
            new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (msg.width, msg.height), 0, (msg.width, msg.height))
            self.K = new_K
            self._build_roi_and_homography()
        except Exception as e:
            self.get_logger().error(f'CameraInfo 解析失败: {e}')

    # -------------------- ROI & Homography（与 APP 一致） --------------------
    def _build_roi_and_homography(self):
        if self.K is None or self._last_T_hand is None or self._white_pose_world is None:
            return
        base_T_cam = self._last_T_hand @ self.hand2cam
        cam_T_base = np.linalg.inv(base_T_cam)
        cam_T_white = cam_T_base @ self._white_pose_world
        Rcw = cam_T_white[:3,:3]; tcw = cam_T_white[:3,3].reshape(3,1)
        H_w2i = self.K @ np.hstack([Rcw[:,0:1], Rcw[:,1:2], tcw])   # world-plane(XY)->image
        self._H_i2w = np.linalg.inv(H_w2i)                           # image->world-plane(XY)

        w2, h2 = self.white_area_size[0]*0.5, self.white_area_size[1]*0.5
        plane = np.array([[ w2,  h2, 1],
                          [-w2,  h2, 1],
                          [-w2, -h2, 1],
                          [ w2, -h2, 1]], dtype=np.float64).T
        img_h = H_w2i @ plane
        img_uv = (img_h[:2,:]/(img_h[2:3,:] + 1e-9)).T
        self._roi_poly = img_uv.astype(np.int32)

    def _pix_to_world_on_plane(self, u:int, v:int):
        """像素(u,v) → 白区平面 XY → 乘 white_area_pose_world → Y 取负 → Z=0.030 → offset/scale → 下垂补偿"""
        if self._H_i2w is None or self._white_pose_world is None:
            return None, None
        vec = self._H_i2w @ np.array([u, v, 1.0], dtype=np.float64)
        vec /= (vec[2] + 1e-9)
        X, Y = float(vec[0]), float(vec[1])
        if self.y_flip: Y = -Y

        Rw = self._white_pose_world[:3,:3]; tw = self._white_pose_world[:3,3]
        p_world = (Rw @ np.array([X, Y, 0.0])) + tw

        # 固定 Z 到平面上方 3cm
        z = self.pick_plane_z

        # offset / scale
        px, py, pz = p_world[0] + self.offset[0], p_world[1] + self.offset[1], z + self.offset[2]
        px, py, pz = px*self.scale[0], py*self.scale[1], pz*self.scale[2]

        # 距离下垂补偿
        r = math.sqrt(px*px + py*py)
        pz += max(0.0, (r - self.droop_r0)/max(1e-6, self.droop_r1)) * self.droop_k

        xyz = [round(px,3), round(py,3), round(pz,3)]
        return xyz, self.world_frame

    # -------------------- 顶视图 warp（用于方块几何） --------------------
    def _warp_white_topdown(self, frame, out_size=(480, 360)):
        if self._roi_poly is None or self._H_i2w is None or self._white_pose_world is None:
            return None, None, None
        base_T_cam = self._last_T_hand @ self.hand2cam
        cam_T_base = np.linalg.inv(base_T_cam)
        cam_T_white = cam_T_base @ self._white_pose_world
        Rcw = cam_T_white[:3,:3]; tcw = cam_T_white[:3,3].reshape(3,1)
        H_w2i = self.K @ np.hstack([Rcw[:,0:1], Rcw[:,1:2], tcw])

        w2, h2 = self.white_area_size[0]*0.5, self.white_area_size[1]*0.5
        plane = np.array([[ w2,  h2, 1],
                          [-w2,  h2, 1],
                          [-w2, -h2, 1],
                          [ w2, -h2, 1]], dtype=np.float64).T
        img_h = H_w2i @ plane
        img_uv = (img_h[:2,:]/(img_h[2:3,:] + 1e-9)).T.astype(np.float32)

        Wpx, Hpx = out_size
        dst = np.array([[Wpx-1, 0],[0,0],[0,Hpx-1],[Wpx-1,Hpx-1]], np.float32)
        H_i2td = cv2.getPerspectiveTransform(img_uv, dst)
        H_td2i = np.linalg.inv(H_i2td)
        topdown = cv2.warpPerspective(frame, H_i2td, (Wpx, Hpx), flags=cv2.INTER_LINEAR)
        return topdown, H_i2td, H_td2i

    # -------------------- core --------------------
    def on_rgb(self, msg: Image):
        now = time.time()
        if (now - self.last_ts) < self.publish_interval or self.K is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_raw = frame.copy()  # YOLO 一直喂原图

        # 只遮挡白区中心的 tag（可选）
        if self.mask_apriltag and self._roi_poly is not None:
            M = cv2.moments(self._roi_poly)
            if M['m00'] > 1e-6:
                cx = int(M['m10']/M['m00']); cy = int(M['m01']/M['m00'])
                cv2.rectangle(frame_raw, (cx-36, cy-36), (cx+36, cy+36), (0,0,0), -1)

        # 1) YOLO 在原图跑
        try:
            yolo_res = self.model(frame_raw, verbose=False)[0]
        except TypeError:
            yolo_res = self.model(frame_raw, verbose=False)[0]

        det = DetectionResult()
        det.image_width  = [frame.shape[1]]
        det.image_height = [frame.shape[0]]
        objects = []
        unix_ts = int(now)

        # ROI test
        def _in_roi(u:int, v:int)->bool:
            if self._roi_poly is None: return True
            return cv2.pointPolygonTest(self._roi_poly.astype(np.float32), (float(u), float(v)), False) >= 0

        # 白名单
        wl = {s.strip().lower() for s in (self.class_whitelist or '').split(',') if s.strip()} if self.use_class_whitelist else set()

        # 2) 处理 YOLO 结果（事后按 ROI + 白名单过滤）
        for b in yolo_res.boxes:
            cls_id  = int(b.cls[0]); raw_cls = str(self.model.names[cls_id]).lower()
            conf    = float(b.conf[0])
            if conf < self.min_conf: continue
            if wl and (raw_cls not in wl): continue

            x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
            x1 = max(0,x1); y1=max(0,y1); x2=min(frame.shape[1]-1,x2); y2=min(frame.shape[0]-1,y2)
            if x2<=x1 or y2<=y1: continue
            u = (x1+x2)//2; v=(y1+y2)//2
            if not _in_roi(u,v): continue

            det.class_name.append(raw_cls)
            det.confidence.append(conf)
            det.center_x.append(int(u))
            det.center_y.append(int(v))

            xyz, xyz_frame = self._pix_to_world_on_plane(u, v)
            det.center_z.append(xyz[2] if xyz else -1.0)
            if xyz is not None:
                objects.append({
                    "id": len(objects)+1,
                    "class_name": raw_cls,
                    "pose": {"frame": xyz_frame, "xyz": xyz, "rpy": [0.0, 0.0, 1.57]},
                    "updated_at": unix_ts,
                    "confidence": round(conf,3)
                })

        # 3) 兜底 cube：在白区顶视图上做 LAB+几何，结果映回像素
        cubes = self._find_cubes_topdown(frame_raw)
        for (uc,vc,w,h,score,ang_deg,H_td2i) in cubes:
            pt = np.array([uc, vc, 1.0], dtype=np.float64)
            uvh = H_td2i @ pt; uvh /= (uvh[2]+1e-9)
            u, v = int(round(uvh[0])), int(round(uvh[1]))
            if not _in_roi(u,v): continue

            det.class_name.append("cube")
            det.confidence.append(float(min(0.99, max(0.5, score))))
            det.center_x.append(int(u))
            det.center_y.append(int(v))

            xyz, xyz_frame = self._pix_to_world_on_plane(u, v)
            det.center_z.append(xyz[2] if xyz else -1.0)
            if xyz is not None:
                objects.append({
                    "id": len(objects)+1,
                    "class_name": "cube",
                    "pose": {"frame": xyz_frame, "xyz": xyz, "rpy": [0.0, 0.0, 1.57]},
                    "angle_deg": round(float(ang_deg),1),
                    "updated_at": unix_ts,
                    "confidence": round(float(min(0.99, max(0.5, score))),3)
                })

        # 4) 发布
        self.pub_det.publish(det)
        if objects:
            self.pub_wm.publish(String(data=json.dumps({"objects": objects}, ensure_ascii=False)))

        # 5) 可视化
        if self.show_window:
            for i, cls in enumerate(det.class_name):
                label = f"{cls} {det.confidence[i]:.2f}".encode('ascii','ignore').decode()
                cv2.circle(frame, (det.center_x[i], det.center_y[i]), 4, (0,255,0), -1)
                cv2.putText(frame, label,
                            (max(det.center_x[i]-40, 5), max(det.center_y[i]-10, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 1)
            if self._roi_poly is not None:
                cv2.polylines(frame, [self._roi_poly], True, (0,255,255), 2)
            cv2.imshow("YOLO", frame); cv2.waitKey(1)

        self.last_ts = now

    # -------------------- cube in topdown --------------------
    def _find_cubes_topdown(self, frame_raw) -> List[Tuple[int,int,int,int,float,float,np.ndarray]]:
        topdown, H_i2td, H_td2i = self._warp_white_topdown(frame_raw, out_size=(480,360))
        if topdown is None: return []

        # 先 LAB，后与顶面掩膜
        lab = cv2.cvtColor(topdown, cv2.COLOR_BGR2LAB)
        if self.only_red and 'red' in self.lab_ranges:
            mn, mx = self.lab_ranges['red']
            mask_lab = cv2.inRange(lab, mn, mx)
        else:
            mask_lab = np.ones(topdown.shape[:2], np.uint8)*255

        gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
        mb = cv2.medianBlur(gray,3); gs = cv2.GaussianBlur(mb,(5,5),5)
        mask_top = cv2.adaptiveThreshold(gs,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY,41,7)
        edges = cv2.Canny(gs,9,41,9,L2gradient=True)
        edges = 255 - cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT,(9,9)))
        mask = cv2.bitwise_and(mask_lab, mask_top)
        mask = cv2.bitwise_and(mask, edges)

        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        props = []
        Htd, Wtd = mask.shape[:2]
        for c in cnts:
            A = cv2.contourArea(c)
            if A < self.min_area_px or A > self.max_area_px:
                continue
            P = cv2.arcLength(c, True)
            circularity = 4.0*np.pi*A / (P*P + 1e-9)
            if circularity > 0.80:  # 防圆：杯口
                continue
            approx = cv2.approxPolyDP(c, 0.02*P, True)
            if len(approx) != 4:
                continue
            if self._right_angle_count(approx) < 3:
                continue

            rect = cv2.minAreaRect(c)
            (cx,cy),(w,h),theta = rect
            if w < 10 or h < 10: continue
            ar = w/float(h)
            if not (0.75 <= ar <= 1.25):  # 顶视里更严格些
                continue

            box = cv2.boxPoints(rect).astype(np.int32)
            x, y, bw, bh = cv2.boundingRect(box)
            fill_ratio = A / float(bw*bh + 1e-6)
            if fill_ratio < 0.80:
                continue

            score = float(min(1.0, A / float(Htd*Wtd)))
            ang   = self._norm_angle(rect)
            props.append((int(round(cx)), int(round(cy)), int(bw), int(bh), score, ang, H_td2i))

        props.sort(key=lambda b: b[2]*b[3], reverse=True)
        return props[:2]

    @staticmethod
    def _norm_angle(rect):
        (cx,cy),(w,h),theta = rect
        ang = float(theta)
        if w < h: ang += 90.0
        if ang > 45.0: ang -= 90.0
        if ang < -45.0: ang += 90.0
        return ang

    @staticmethod
    def _right_angle_count(poly: np.ndarray) -> int:
        pts = poly.reshape(-1,2)
        right = 0
        for i in range(4):
            v1 = pts[(i+1)%4] - pts[i]
            v2 = pts[(i-1)%4] - pts[i]
            c = np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2) + 1e-9)
            ang = abs(np.degrees(np.arccos(np.clip(c, -1, 1))))
            if 70 <= ang <= 110:
                right += 1
        return right


# ---------------------- main ----------------------
def main(args=None):
    rclpy.init(args=args)
    node = AppCompatibleYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try: cv2.destroyAllWindows()
        except Exception: pass


if __name__ == '__main__':
    main()

