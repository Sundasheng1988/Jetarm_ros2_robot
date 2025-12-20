import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

JOINT_LIMITS = {
    'joint1': (-2.09, 2.09),
    'joint2': (-2.09, 2.09),
    'joint3': (-2.09, 2.09),
    'joint4': (-2.09, 2.09),
    'joint5': (-2.09, 2.09),
}

def rad_to_pulse(joint_name, rad):
    rad_min, rad_max = JOINT_LIMITS[joint_name]
    pulse_min, pulse_max = 0, 1000
    rad = max(rad_min, min(rad_max, rad))
    scale = (pulse_max - pulse_min) / (rad_max - rad_min)
    return int(pulse_min + (rad - rad_min) * scale)

class StateToPulseNode(Node):
    def __init__(self):
        super().__init__('state_to_pulse_node')

        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/state',
            self.listener_callback,
            10)

        self.get_logger().info('✅ State → Pulse 节点启动成功！监听 /arm_controller/state ...')

    def listener_callback(self, msg: JointTrajectoryControllerState):
        # 用 desired 或 actual 都可以，通常 desired 是目标位置
        names = msg.joint_names
        desired_positions = msg.desired.positions
        actual_positions = msg.actual.positions

        pulses_desired = []
        for name, rad in zip(names, desired_positions):
            if name in JOINT_LIMITS:
                pulse = rad_to_pulse(name, rad)
                pulses_desired.append((name, pulse))
                self.get_logger().info(
                    f'[Desired] {name}: {rad:.2f} rad → {pulse} pulse')

        pulses_actual = []
        for name, rad in zip(names, actual_positions):
            if name in JOINT_LIMITS:
                pulse = rad_to_pulse(name, rad)
                pulses_actual.append((name, pulse))
                self.get_logger().info(
                    f'[Actual] {name}: {rad:.2f} rad → {pulse} pulse')

        self.get_logger().info(f'🚩 Desired 脉冲组: {pulses_desired}')
        self.get_logger().info(f'🚩 Actual 脉冲组: {pulses_actual}')

def main(args=None):
    rclpy.init(args=args)
    node = StateToPulseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

