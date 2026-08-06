#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


SAVE_DIR = os.path.expanduser(
    "~/ros2_ws/tools/vision_roi_tuner/inputs/images"
)

TOPIC = "/depth_cam/rgb/image_raw"


class DatasetCaptureNode(Node):

    def __init__(self):
        super().__init__("dataset_capture")

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.bridge = CvBridge()
        self.frame = None
        self.counter = self._find_last_index()

        self.create_subscription(
            Image,
            TOPIC,
            self.on_image,
            10,
        )

        self.get_logger().info(
            f"\n"
            f"========== DATASET CAPTURE ==========\n"
            f"SAVE: {SAVE_DIR}\n"
            f"S → Save image\n"
            f"Q → Exit\n"
            f"===================================="
        )

    def _find_last_index(self):

        files = [
            f
            for f in os.listdir(SAVE_DIR)
            if f.startswith("capture_")
        ]

        if not files:
            return 1

        nums = []

        for f in files:
            try:
                nums.append(
                    int(
                        f.replace(
                            "capture_",
                            ""
                        ).replace(
                            ".jpg",
                            ""
                        )
                    )
                )
            except:
                pass

        return max(nums) + 1

    def on_image(self, msg):

        try:
            self.frame = self.bridge.imgmsg_to_cv2(
                msg,
                "bgr8"
            )

        except Exception as e:
            self.get_logger().error(str(e))

    def save_current(self):

        if self.frame is None:
            print("No image")
            return

        name = (
            f"capture_"
            f"{self.counter:04d}.jpg"
        )

        path = os.path.join(
            SAVE_DIR,
            name
        )

        cv2.imwrite(
            path,
            self.frame
        )

        print(f"\nSaved:\n{path}\n")

        self.counter += 1

    def loop(self):

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.03
            )

            if self.frame is None:
                continue

            show = self.frame.copy()

            cv2.putText(
                show,
                "S=Save  Q=Quit",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.imshow(
                "Vision Dataset Capture",
                show
            )

            key = cv2.waitKey(1)

            if key == ord("s"):
                self.save_current()

            elif key == ord("q"):
                break

        cv2.destroyAllWindows()


def main():

    rclpy.init()

    node = DatasetCaptureNode()

    try:
        node.loop()

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()