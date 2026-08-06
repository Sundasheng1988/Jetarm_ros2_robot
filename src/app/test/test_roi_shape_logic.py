import sys
import unittest.mock as mock
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

mock_rclpy = mock.MagicMock()
mock_node = mock.MagicMock()
mock_msg = mock.MagicMock()
mock_qos = mock.MagicMock()
mock_cv2 = mock.MagicMock()
mock_yaml = mock.MagicMock()
mock_cvbridge = mock.MagicMock()

sys.modules["rclpy"] = mock_rclpy
sys.modules["rclpy.node"] = mock_node
sys.modules["std_msgs"] = mock_msg
sys.modules["std_msgs.msg"] = mock_msg
sys.modules["sensor_msgs"] = mock_msg
sys.modules["sensor_msgs.msg"] = mock_msg
sys.modules["geometry_msgs"] = mock_msg
sys.modules["geometry_msgs.msg"] = mock_msg
sys.modules["vision_interfaces"] = mock_msg
sys.modules["vision_interfaces.msg"] = mock_msg
sys.modules["rclpy.qos"] = mock_qos
sys.modules["cv_bridge"] = mock_cvbridge
sys.modules["cv2"] = mock_cv2
mock_cv2.findContours.return_value = ([], None)
mock_cv2.Rodrigues.return_value = (np.zeros(3), np.zeros(3))
mock_cv2.projectPoints.return_value = (np.zeros((4, 1, 2)), None)
sys.modules["yaml"] = mock.MagicMock()

from app.roi_color_detector_node import right_angle_count


def test_right_angle_count_square():
    poly = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.int32)
    assert right_angle_count(poly) == 4


def test_right_angle_count_diamond():
    poly = np.array([[5, 0], [10, 5], [5, 10], [0, 5]], dtype=np.int32)
    assert right_angle_count(poly) == 4


def test_right_angle_count_rectangle():
    poly = np.array([[0, 0], [20, 0], [20, 10], [0, 10]], dtype=np.int32)
    assert right_angle_count(poly) == 4


def test_right_angle_count_trapezoid():
    poly = np.array([[0, 0], [20, 0], [15, 8], [5, 8]], dtype=np.int32)
    assert right_angle_count(poly) == 0


def test_right_angle_count_parallelogram():
    poly = np.array([[0, 0], [10, 0], [12, 5], [2, 5]], dtype=np.int32)
    cnt = right_angle_count(poly)
    assert cnt <= 1
