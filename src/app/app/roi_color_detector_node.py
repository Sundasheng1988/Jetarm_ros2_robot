#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, json
from pathlib import Path
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from vision_interfaces.msg import DetectionResult
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

qos_sensor = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

def rodrigues_rt_from_T(T: np.ndarray):
    R = T[:3, :3]; t = T[:3, 3].reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R)
    return rvec, t

def right_angle_count(poly: np.ndarray) -> int:
    """Count near-right angles in an approximated contour polygon.

    cv2.approxPolyDP() can return 3, 4, or more points. The previous
    implementation assumed exactly 4 points and crashed on triangles.
    """
    pts = poly.reshape(-1, 2)
    n = len(pts)
    if n < 4:
        return 0

    right = 0
    for i in range(n):
        v1 = pts[(i + 1) % n] - pts[i]
        v2 = pts[(i - 1) % n] - pts[i]
        norm = np.linalg.norm(v1) * np.linalg.norm(v2)
        if norm < 1e-9:
            continue
        c = np.dot(v1, v2) / norm
        ang = abs(np.degrees(np.arccos(np.clip(c, -1, 1))))
        if 70 <= ang <= 110:
            right += 1
    return right

def rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
    cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
    cy = math.cos(yaw*0.5);   sy = math.sin(yaw*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx, qy, qz, qw)

class RoiColorDetectorNode(Node):
    def __init__(self):
        super().__init__('roi_color_detector_node')

        self.declare_parameter('rgb_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_cam/rgb/camera_info')
        self.declare_parameter('publish_interval', 0.30)
        self.declare_parameter('show_window', True)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('debug_window_scale', 1.0)
        self.declare_parameter('image_result_topic', '/roi_color_detector/image_result')
        self.declare_parameter('world_frame', 'base')

        self.declare_parameter('transform_yaml', '/home/sundasheng/ros2_ws/src/app/config/transform.yaml')
        self.declare_parameter('white_area_size', [0.175, 0.135])
        self.declare_parameter('z_up', 0.030)
        self.declare_parameter('y_flip', False)
        self.declare_parameter('offset', [0.0, 0.0, 0.0])
        self.declare_parameter('scale',  [1.0, 1.0, 1.0])
        self.declare_parameter('droop_k', 0.025)
        self.declare_parameter('droop_r0', 0.15)
        self.declare_parameter('droop_r1', 0.20)

        self.declare_parameter('lab_config', '/home/sundasheng/ros2_ws/src/app/config/lab_config.yaml')
        self.declare_parameter('color_keys', ['red', 'blue'])

        self.declare_parameter('min_area_px', 500)
        self.declare_parameter('max_area_px', 30000)

        self.declare_parameter('ball_colors', ['purple'])
        self.declare_parameter('ball_circ', 0.82)
        self.declare_parameter('cyl_circ',  0.72)
        self.declare_parameter('cup_min_size_px', 90)
        self.declare_parameter('use_shading_ball', True)
        self.declare_parameter('shading_thresh', 8.0)

        g = self.get_parameter
        self.rgb_topic         = g('rgb_topic').value
        self.camera_info_topic = g('camera_info_topic').value
        self.publish_interval  = float(g('publish_interval').value)
        self.show_window       = bool(g('show_window').value)
        self.show_debug_window = bool(g('show_debug_window').value)
        self.debug_window_scale = float(g('debug_window_scale').value)
        self.image_result_topic = g('image_result_topic').value
        self.world_frame       = g('world_frame').value

        self.transform_yaml  = str(g('transform_yaml').value)
        self.white_area_size = [float(x) for x in g('white_area_size').value]
        self.z_up            = float(g('z_up').value)
        self.y_flip          = bool(g('y_flip').value)
        self.offset          = [float(x) for x in g('offset').value]
        self.scale           = [float(x) for x in g('scale').value]
        self.droop_k         = float(g('droop_k').value)
        self.droop_r0        = float(g('droop_r0').value)
        self.droop_r1        = float(g('droop_r1').value)

        self.lab_config_path = str(g('lab_config').value)
        self.color_keys      = [str(x) for x in g('color_keys').value]
        self.min_area_px     = int(g('min_area_px').value)
        self.max_area_px     = int(g('max_area_px').value)

        self.ball_colors      = [str(x) for x in self.get_parameter('ball_colors').value]
        self.ball_circ        = float(self.get_parameter('ball_circ').value)
        self.cyl_circ         = float(self.get_parameter('cyl_circ').value)
        self.cup_min_size_px  = int(self.get_parameter('cup_min_size_px').value)
        self.use_shading_ball = bool(self.get_parameter('use_shading_ball').value)
        self.shading_thresh   = float(self.get_parameter('shading_thresh').value)

        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, self.image_result_topic, qos_sensor)
        self.pub_det = self.create_publisher(DetectionResult, '/roi_vision_target', qos_sensor)
        self.pub_wm  = self.create_publisher(String, '/world_model/roi_objects', qos_sensor)
        self.pub_pose_array   = self.create_publisher(PoseArray,  '/roi_objects/poses', qos_sensor)
        self.pub_best_pose    = self.create_publisher(PoseStamped,'/roi_target_pose',   qos_sensor)

        self.create_subscription(Image,      self.rgb_topic,         self.on_rgb,     qos_sensor)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_caminfo, qos_sensor)

        self.K=None; self.D=None
        self._roi_poly=None
        self._white_world=None
        self._white_cam=None
        self._proj_r=None; self._proj_t=None

        self._load_transform()
        self.lab_ranges = self._load_lab_ranges()
        self.last_ts = 0.0
        self._headless_warned = False

        if self.show_debug_window:
            try:
                cv2.namedWindow("ROI Color Detection", cv2.WINDOW_NORMAL)
            except Exception:
                if not self._headless_warned:
                    self.get_logger().warn("could not create OpenCV window — running headless")
                    self._headless_warned = True

        self.get_logger().info('✅ roi_color_detector_node 已启动（仿 APP 检测）')

    def _load_transform(self):
        p = Path(self.transform_yaml)
        if not p.exists():
            self.get_logger().warn(f'transform.yaml 不存在: {p}')
            return
        try:
            cfg = yaml.safe_load(p.read_text())
            self._white_world = np.array(cfg['white_area_pose_world'], dtype=np.float64).reshape(4,4)
            self._white_cam   = np.array(cfg['white_area_pose_cam'  ], dtype=np.float64).reshape(4,4)
            self._proj_r, self._proj_t = rodrigues_rt_from_T(self._white_cam)
        except Exception as e:
            self.get_logger().error(f'解析 transform.yaml 失败: {e}')

    def _load_lab_ranges(self):
        rng = {}
        try:
            data = yaml.safe_load(Path(self.lab_config_path).read_text())
            cl = data['/**']['ros__parameters']['color_range_list']
            for name in cl:
                rng[name] = (tuple(cl[name]['min']), tuple(cl[name]['max']))
            self.color_keys = list(cl.keys())
            self.get_logger().info(f"✅ 自动加载颜色: {self.color_keys}")
        except Exception as e:
            self.get_logger().warn(f'lab_config 读取失败，使用默认阈值: {e}')
            rng['red']  = ((0,140,120),(255,195,200))
            rng['blue'] = ((0,120, 50),(255,180,125))
            self.color_keys = ['red','blue']
        return rng

    def on_caminfo(self, msg: CameraInfo):
        try:
            self.K=np.array(msg.k, dtype=np.float64).reshape(3,3)
            self.D=np.array(msg.d, dtype=np.float64).reshape(-1,1) if msg.d else np.zeros((5,1))
            self._build_roi()
        except Exception as e:
            self.get_logger().error(f'CameraInfo 解析失败: {e}')

    def _build_roi(self):
        if self.K is None or self._proj_r is None or self._proj_t is None:
            return
        w2, h2 = self.white_area_size[0]*0.5, self.white_area_size[1]*0.5
        pts = np.array([
            [ w2,  h2, 0.0],
            [-w2,  h2, 0.0],
            [-w2, -h2, 0.0],
            [ w2, -h2, 0.0],
        ], dtype=np.float64).reshape(-1,1,3)
        imgpts,_ = cv2.projectPoints(pts, self._proj_r, self._proj_t, self.K, self.D)
        self._roi_poly = imgpts.reshape(-1,2).astype(np.int32)

    def _in_roi(self, u:int, v:int)->bool:
        if self._roi_poly is None: return True
        return cv2.pointPolygonTest(self._roi_poly.astype(np.float32), (float(u),float(v)), False) >= 0

    def _sphere_shading_score(self, gray_img: np.ndarray, u: int, v: int, r: float) -> float:
        H, W = gray_img.shape[:2]
        r = max(8.0, min(r, 80.0))
        r_center = int(0.35 * r)
        r_ring1  = int(0.55 * r)
        r_ring2  = int(0.85 * r)
        if not (0 <= u < W and 0 <= v < H): return 0.0
        Y, X = np.ogrid[:H, :W]
        dist2 = (X - u)**2 + (Y - v)**2
        m_center = dist2 <= (r_center*r_center)
        m_ring   = (dist2 >= (r_ring1*r_ring1)) & (dist2 <= (r_ring2*r_ring2))
        if m_center.sum() < 10 or m_ring.sum() < 10: return 0.0
        c = float(gray_img[m_center].mean())
        o = float(gray_img[m_ring].mean())
        return c - o

    def _dominant_color_key(self, lab_img: np.ndarray, contour: np.ndarray) -> str:
        mask = np.zeros(lab_img.shape[:2], np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
        best_key, best_hits = None, -1
        for k, (mn, mx) in self.lab_ranges.items():
            m = cv2.inRange(lab_img, np.array(mn, np.uint8), np.array(mx, np.uint8))
            hits = int((m & mask).sum() // 255)
            if hits > best_hits:
                best_hits, best_key = hits, k
        return best_key or "unknown"

    def _pix_to_world(self, u:int, v:int):
        if self.K is None or self._white_world is None or self._white_cam is None:
            return None
        fx, fy = self.K[0,0], self.K[1,1]
        cx, cy = self.K[0,2], self.K[1,2]
        x = (u-cx)/fx; y = (v-cy)/fy

        n  = self._white_cam[:3,2]
        p0 = self._white_cam[:3,3]
        ray = np.array([x,y,1.0], dtype=np.float64)
        denom = np.dot(n, ray)
        if abs(denom) < 1e-8: return None
        t = np.dot(n, p0)/denom
        p_cam = t*ray

        white_T_cam = np.linalg.inv(self._white_cam)
        p_white = white_T_cam[:3,:3] @ p_cam + white_T_cam[:3,3]
        X, Y = float(p_white[0]), float(p_white[1])
        if self.y_flip: Y = -Y

        Rw = self._white_world[:3,:3]; tw = self._white_world[:3,3]
        p_world_flat = (Rw @ np.array([X, Y, 0.0])) + tw
        px, py, pz = p_world_flat[0], p_world_flat[1], self.z_up

        px = (px + self.offset[0]) * self.scale[0]
        py = (py + self.offset[1]) * self.scale[1]
        pz = (pz + self.offset[2]) * self.scale[2]
        r = math.sqrt(px*px + py*py)
        pz += max(0.0, (r - self.droop_r0)/max(1e-6, self.droop_r1)) * self.droop_k
        return [round(px, 3), round(py, 3), round(pz, 3)]

    def _pix_to_white_xy(self, u:int, v:int):
        if self.K is None or self._white_cam is None:
            return None
        fx, fy = self.K[0,0], self.K[1,1]
        cx, cy = self.K[0,2], self.K[1,2]
        x = (u-cx)/fx; y = (v-cy)/fy
        n  = self._white_cam[:3,2]
        p0 = self._white_cam[:3,3]
        ray = np.array([x,y,1.0], dtype=np.float64)
        denom = float(np.dot(n, ray))
        if abs(denom) < 1e-8:
            return None
        t = float(np.dot(n, p0))/denom
        p_cam = t*ray
        white_T_cam = np.linalg.inv(self._white_cam)
        p_white = white_T_cam[:3,:3] @ p_cam + white_T_cam[:3,3]
        X, Y = float(p_white[0]), float(p_white[1])
        if self.y_flip:
            Y = -Y
        return X, Y

    def on_rgb(self, msg: Image):
        now = time.time()
        if (now - self.last_ts) < self.publish_interval:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        H, W = frame.shape[:2]

        vis = frame.copy()
        if self._roi_poly is not None:
            cv2.polylines(vis, [self._roi_poly], True, (0,255,255), 2)
            M = cv2.moments(self._roi_poly)
            if M['m00'] > 1e-6:
                cx = int(M['m10']/M['m00']); cy = int(M['m01']/M['m00'])
                cv2.drawMarker(vis, (cx,cy), (0,255,255), cv2.MARKER_TILTED_CROSS, 18, 2)

        mask_roi = np.zeros((H,W), np.uint8)
        if self._roi_poly is not None:
            cv2.fillPoly(mask_roi, [self._roi_poly], 255)
        roi_img = cv2.bitwise_and(frame, frame, mask=mask_roi)

        lab  = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)

        det = DetectionResult()
        det.image_width  = [W]
        det.image_height = [H]

        objects=[]; unix_ts=int(now)
        poses: list[tuple[Pose,float]] = []

        for color in self.color_keys:
            if color not in self.lab_ranges:
                continue
            mn, mx = self.lab_ranges[color]
            mask = cv2.inRange(lab, np.array(mn, np.uint8), np.array(mx, np.uint8))
            mask = cv2.medianBlur(mask, 3)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3,3),np.uint8), iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8), iterations=2)

            cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                A = cv2.contourArea(c)
                if A < self.min_area_px or A > self.max_area_px:
                    continue
                P = cv2.arcLength(c, True)
                rect = cv2.minAreaRect(c)
                (cx,cy),(w,h),theta = rect
                if w < 12 or h < 12:
                    continue
                u, v = int(round(cx)), int(round(cy))
                if not self._in_roi(u, v):
                    continue

                box  = cv2.boxPoints(rect).astype(np.int32)
                circ = 4.0*np.pi*A/(P*P + 1e-9)

                target_cls = None
                approx_poly = cv2.approxPolyDP(c, 0.02 * P, True)
                ar = w / float(h)
                right_cnt = right_angle_count(approx_poly)

                if right_cnt >= 3 and 0.70 <= ar <= 1.35:
                    target_cls = 'cube'
                elif circ >= 0.60 and min(w, h) >= self.cup_min_size_px:
                    target_cls = 'cup'
                else:
                    is_ball_by_color = (color in self.ball_colors and circ >= self.ball_circ)
                    shading_ok = False
                    if self.use_shading_ball and circ >= self.cyl_circ:
                        r_est = min(w, h) * 0.5
                        shade_score = self._sphere_shading_score(gray, u, v, r_est)
                        shading_ok = (shade_score >= self.shading_thresh)
                    if is_ball_by_color or shading_ok:
                        target_cls = 'ball'
                    else:
                        if circ >= self.cyl_circ:
                            target_cls = 'cylinder'
                if target_cls is None:
                    continue

                true_color = self._dominant_color_key(lab, c)
                self.get_logger().debug(
                    f"detect: {target_cls} {true_color} "
                    f"conf={float(min(0.99, max(0.50, circ))):.2f} "
                    f"circ={circ:.3f} ar={ar:.2f} right_cnt={right_cnt}"
                )
                xyz = self._pix_to_world(u, v)

                # 估计世界系 yaw
                yaw_world = 1.57
                try:
                    p0_xy = self._pix_to_white_xy(int(box[0][0]), int(box[0][1]))
                    p1_xy = self._pix_to_white_xy(int(box[1][0]), int(box[1][1]))
                    if p0_xy is not None and p1_xy is not None and self._white_world is not None:
                        dx_w, dy_w = (p1_xy[0] - p0_xy[0]), (p1_xy[1] - p0_xy[1])
                        Rw = self._white_world[:3,:3]
                        d_world = Rw @ np.array([dx_w, dy_w, 0.0])
                        yaw_world = math.atan2(float(d_world[1]), float(d_world[0]))
                except Exception:
                    pass

                det.class_name.append(target_cls)
                det.center_x.append(u)
                det.center_y.append(v)
                det.center_z.append(xyz[2] if xyz else -1.0)
                det.confidence.append(float(min(0.99, max(0.50, circ))))

                if xyz:
                    rpy = (0.0, 0.0, float(yaw_world))
                    qx,qy,qz,qw = rpy_to_quat(*rpy)
                    objects.append({
                        "id": len(objects)+1,
                        "class_name": target_cls,
                        "color": true_color,
                        "pose": {"frame": self.world_frame, "xyz": xyz, "rpy": [round(rpy[0],3), round(rpy[1],3), round(rpy[2],3)]},
                        "updated_at": unix_ts,
                        "confidence": round(float(min(0.99, max(0.50, circ))),3)
                    })
                    p = Pose()
                    p.position.x, p.position.y, p.position.z = xyz
                    p.orientation.x = qx; p.orientation.y = qy
                    p.orientation.z = qz; p.orientation.w = qw
                    poses.append((p, det.confidence[-1]))

                cv2.polylines(vis, [box], True, (255,255,0), 2)
                cv2.circle(vis, (u,v), 5, (0,0,0), -1)

                if self.show_debug_window:
                    conf_val = float(min(0.99, max(0.50, circ)))
                    xyz_str = ""
                    if xyz:
                        xyz_str = f" xyz:[{xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}]"
                    label = f"{target_cls} {true_color} {conf_val:.2f}{xyz_str}"
                    cv2.putText(vis, label, (u + 8, v - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1,
                                cv2.LINE_AA)

        self.pub_det.publish(det)
        if objects:
            self.pub_wm.publish(String(data=json.dumps({"objects": objects}, ensure_ascii=False)))

        if poses:
            pa = PoseArray()
            pa.header.stamp = self.get_clock().now().to_msg()
            pa.header.frame_id = self.world_frame
            pa.poses = [p for p,_ in poses]
            self.pub_pose_array.publish(pa)

            best_pose, _ = max(poses, key=lambda t: t[1])
            ps = PoseStamped()
            ps.header = pa.header
            ps.pose   = best_pose
            self.pub_best_pose.publish(ps)

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))

        if self.show_debug_window:
            try:
                display = vis
                if self.debug_window_scale != 1.0:
                    display = cv2.resize(
                        vis, None,
                        fx=self.debug_window_scale,
                        fy=self.debug_window_scale,
                    )
                cv2.imshow("ROI Color Detection", display)
                cv2.waitKey(1)
            except Exception:
                if not self._headless_warned:
                    self.get_logger().warn(
                        "cv2.imshow failed — running headless, no debug window"
                    )
                    self._headless_warned = True

        self.last_ts = now

def main(args=None):
    rclpy.init(args=args)
    node = RoiColorDetectorNode()
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
