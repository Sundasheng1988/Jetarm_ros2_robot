import rclpy
from rclpy.node import Node
import numpy as np

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        # 外参矩阵（从 transform.yaml 读取）
        self.extristric = np.array([
            [-0.9993036287686337, -0.03018707496119184, -0.016402029757947894, 0.1711312004852799],
            [-0.030850338029154056, 0.9988107511842681, 0.03573626608173539, 0.0036404146489908517],
            [0.015300767264099903, 0.03622163544021059, -0.9990469094896739, -0.019623799869184333],
            [0.0, 0.0, 0.0, 1.0]
        ])

        self.hand2cam_transformation_matrix = np.array([
            [-0.01782217674806108, -0.1046008609006226, 0.9943545795714974, -0.09630850772104702],
            [-0.9998308045282818, 0.006393502312188604, -0.01724776636300579, -0.009750665301640829],
            [-0.004553277093442498, -0.9944937320199825, -0.10469710903686735, 0.07899833508544073],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # 相机内参（从 camera_info.yaml 中读取）
        self.fx = 452.7542419433594
        self.fy = 452.7542419433594
        self.cx = 324.42041015625
        self.cy = 238.51194763183594

        # 已知的像素坐标和深度值
        self.u = 335
        self.v = 270
        self.z_raw = 203.0  # 单位为毫米
        self.z = self.z_raw / 1000.0  # 转换为米

        # 计算并打印结果
        self.calculate_and_print()

    def calculate_and_print(self):
        # 步骤 1: 计算相机坐标系中的 (x, y, z)
        x = (self.u - self.cx) * self.z / self.fx
        y = (self.v - self.cy) * self.z / self.fy
        cam_point = np.array([x, y, self.z, 1.0]).reshape(4, 1)

        # 步骤 2: 使用 hand2cam 外参将相机坐标转换为手坐标
        hand_point = self.hand2cam_transformation_matrix @ cam_point

        # 步骤 3: 使用 extristric 外参将手坐标转换为世界坐标
        world_point = self.extristric @ hand_point
        xyz = world_point[:3].flatten()

        # 输出计算结果
        self.get_logger().info(f"Tag 在相机坐标系中的位置: x={x:.6f}, y={y:.6f}, z={self.z:.6f} 米")
        self.get_logger().info(f"Tag 在手坐标系中的位置: {hand_point[:3].flatten()}")
        self.get_logger().info(f"Tag 在机械臂世界坐标系中的位置: {xyz}")

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

