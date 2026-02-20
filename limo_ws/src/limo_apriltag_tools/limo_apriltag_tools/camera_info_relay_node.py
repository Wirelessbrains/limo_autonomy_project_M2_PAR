import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class CameraInfoRelay(Node):
    def __init__(self):
        super().__init__('camera_info_relay')

        self.declare_parameter('input_topic', '/rgb_camera/camera_info')
        self.declare_parameter('output_topic', '/camera_info')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(CameraInfo, output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            CameraInfo,
            input_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Relay active: {input_topic} -> {output_topic}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
