#!/usr/bin/env python3
import argparse
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


def apply_curve(value: float, exponent: float, gain: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) ** exponent) * gain


class CmdVelCurve(Node):
    def __init__(self, args):
        super().__init__('cmd_vel_curve')
        self.linear_gain = args.linear_gain
        self.angular_gain = args.angular_gain
        self.linear_exponent = args.linear_exponent
        self.angular_exponent = args.angular_exponent
        self.linear_deadzone = args.linear_deadzone
        self.angular_deadzone = args.angular_deadzone
        self.in_topic = args.input
        self.out_topic = args.output

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, self.out_topic, qos)
        self.sub = self.create_subscription(
            Twist,
            self.in_topic,
            self._on_cmd,
            qos,
        )

        self.get_logger().info(
            f"CmdVel curve: in={self.in_topic} out={self.out_topic} "
            f"lin_gain={self.linear_gain} lin_exp={self.linear_exponent} "
            f"ang_gain={self.angular_gain} ang_exp={self.angular_exponent} "
            f"lin_dz={self.linear_deadzone} ang_dz={self.angular_deadzone}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        out = Twist()
        out.linear.x = apply_curve(
            msg.linear.x, self.linear_exponent, self.linear_gain, self.linear_deadzone
        )
        out.linear.y = apply_curve(
            msg.linear.y, self.linear_exponent, self.linear_gain, self.linear_deadzone
        )
        out.linear.z = apply_curve(
            msg.linear.z, self.linear_exponent, self.linear_gain, self.linear_deadzone
        )
        out.angular.x = apply_curve(
            msg.angular.x, self.angular_exponent, self.angular_gain, self.angular_deadzone
        )
        out.angular.y = apply_curve(
            msg.angular.y, self.angular_exponent, self.angular_gain, self.angular_deadzone
        )
        out.angular.z = apply_curve(
            msg.angular.z, self.angular_exponent, self.angular_gain, self.angular_deadzone
        )
        self.pub.publish(out)


def parse_args():
    parser = argparse.ArgumentParser(description="Apply curve/gain to cmd_vel.")
    parser.add_argument("--input", default="/cmd_vel_raw", help="Input cmd_vel topic.")
    parser.add_argument("--output", default="/model/limo_01/cmd_vel", help="Output cmd_vel topic.")
    parser.add_argument("--linear-gain", type=float, default=1.0, help="Linear gain.")
    parser.add_argument("--angular-gain", type=float, default=2.5, help="Angular gain.")
    parser.add_argument("--linear-exponent", type=float, default=1.0, help="Linear exponent.")
    parser.add_argument("--angular-exponent", type=float, default=1.0, help="Angular exponent.")
    parser.add_argument("--linear-deadzone", type=float, default=0.03, help="Linear deadzone.")
    parser.add_argument("--angular-deadzone", type=float, default=0.05, help="Angular deadzone.")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CmdVelCurve(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
