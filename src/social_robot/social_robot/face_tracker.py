#!/usr/bin/env python3
# coding: utf8

import cv2
import mediapipe as mp
import numpy as np

# ★ 正确导入 PID 类与 common 工具函数（对应 social_robot/sdk）
from social_robot.sdk.pid import PID
from social_robot.sdk.common import (
    show_faces,
    mp_face_location,
    box_center,
    distance,
)


class FaceTracker:
    def __init__(self):
        self.tracker_type = "face"

        # mediapipe 人脸检测器
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=0.5,
        )

        # ★ 原厂 PID 参数（完全一致）
        self.pid_yaw = PID(38.0, 0, 1.2)
        self.pid_pitch = PID(32.0, 0, 1.2)

        # ★ 原厂用于稳定检测的计数器
        self.detected_face = 0

        # ★ 初始舵机值（由 goback 设置，但 tracker 仍需保持内部状态）
        self.yaw = 500      # ID1 左右舵机居中
        self.pitch = 350    # ID4 水平抬头姿态（原厂姿势）

    # ------------------------------------------------------
    # 主处理函数：输入图像 → 输出 (pitch, yaw)
    # ------------------------------------------------------
    def proc(self, source_image, result_image):

        # mediapipe 处理图像（RGB）
        results = self.face_detector.process(source_image)

        # 获取人脸 bounding box
        boxes, keypoints = mp_face_location(results, source_image)

        h, w = source_image.shape[:2]

        if len(boxes) > 0:
            # 连续检测计数器，避免抖动
            self.detected_face = min(self.detected_face + 1, 20)

            if self.detected_face >= 5:
                # 计算所有人脸中心
                centers = [box_center(b) for b in boxes]
                dists = [distance(c, (w / 2, h / 2)) for c in centers]

                # 选取距离画面中心最近的人脸
                box, center, _ = min(
                    zip(boxes, centers, dists),
                    key=lambda x: x[2]
                )
                
                cx, cy = center
                dx = cx / w
                dy = cy / h
                
                # ★★★★★ 调试日志（核心） ★★★★★
                print("\n=== FACE TRACK DEBUG ===")
                print(f"center = ({cx:.1f}, {cy:.1f})  normalized = ({dx:.3f}, {dy:.3f})")
                print(f"dy_err={dy-0.5:.3f}  dx_err={dx-0.5:.3f}")
                print(f"before update: pitch={self.pitch}, yaw={self.yaw}")

                # --- pitch (上下) ---
                if abs(dy - 0.5) > 0.01:
                    self.pid_pitch.SetPoint = 0.5
                    self.pid_pitch.update(dy)
                    self.pitch += self.pid_pitch.output
                else:
                    self.pid_pitch.clear()

                # --- yaw (左右) ---
                if abs(dx - 0.5) > 0.01:
                    self.pid_yaw.SetPoint = 0.5
                    self.pid_yaw.update(dx)
                    self.yaw += self.pid_yaw.output
                else:
                    self.pid_yaw.clear()

                # 舵机范围限制（原厂）
                self.pitch = int(max(100, min(740, self.pitch)))
                self.yaw   = int(max(0,   min(1000, self.yaw)))

        else:
            # 没检测到人脸时计数清零
            if self.detected_face > 0:
                self.detected_face -= 1
            else:
                self.pid_pitch.clear()
                self.pid_yaw.clear()

        # 显示人脸位置
        result_image = show_faces(source_image, result_image, boxes, keypoints)

        # 返回图像和两个舵机脉冲值
        return result_image, (self.pitch, self.yaw)

