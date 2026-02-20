from collections import deque
from typing import Deque, List, Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import UInt8MultiArray


class ParkingSpotIndicator(Node):
    """Aggregate contact sensors with debounce and publish spot occupancy."""

    def __init__(self) -> None:
        super().__init__('parking_spot_indicator')
        self.spot_positions: List[Tuple[float, float]] = [
            (-2.0, 1.9),
            (0.0, 1.9),
            (2.0, 1.9),
            (-2.0, -1.9),
            (0.0, -1.9),
            (2.0, -1.9),
        ]
        self.visible_z = self.declare_parameter('visible_z', 0.90).value
        self.hidden_z = self.declare_parameter('hidden_z', -1.0).value
        self.publish_rate = max(self.declare_parameter('publish_rate', 1.0).value, 0.1)
        self.consecutive_required = max(int(self.declare_parameter('consecutive_required', 3).value), 1)
        self.set_pose_service = self.declare_parameter(
            'set_pose_service', '/world/limo_parking_lot/set_pose'
        ).value

        self.occupancy: List[int] = [0] * len(self.spot_positions)
        self.last_green_z: List[float | None] = [None] * len(self.spot_positions)
        self.last_red_z: List[float | None] = [None] * len(self.spot_positions)
        self.windows: List[Deque[bool]] = [deque(maxlen=self.consecutive_required) for _ in self.spot_positions]
        self.status_pub = self.create_publisher(UInt8MultiArray, '/parking/spot_status', 10)

        self.set_pose_cli = self.create_client(SetEntityPose, self.set_pose_service)
        self._wait_for_service()

        for idx in range(len(self.spot_positions)):
            short_topic = f'/parking/spot_0{idx + 1}/contact'
            world_topic = (
                f'/world/limo_parking_lot/model/parking_sensors/link/'
                f'spot_0{idx + 1}_zone/sensor/spot_0{idx + 1}_sensor/contact'
            )
            self.create_subscription(Contacts, short_topic, self._make_contact_cb(idx), qos_profile_sensor_data)
            self.create_subscription(Contacts, world_topic, self._make_contact_cb(idx), qos_profile_sensor_data)

        self._publish_status()
        self._update_indicators(force=True)
        self.create_timer(1.0 / self.publish_rate, self._heartbeat)
        self.get_logger().info('ParkingSpotIndicator running.')

    def _wait_for_service(self) -> None:
        if self.set_pose_cli.service_is_ready():
            return
        self.get_logger().info(
            f'Waiting for service {self.set_pose_service} to move indicators...'
        )
        while rclpy.ok() and not self.set_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_pose...')

    def _make_contact_cb(self, idx: int):
        def cb(msg: Contacts) -> None:
            has_contact = False
            if hasattr(msg, 'states'):
                has_contact = self._contact_has_vehicle(getattr(msg, 'states'))
            elif hasattr(msg, 'contacts'):
                has_contact = self._contact_has_vehicle(getattr(msg, 'contacts'))

            win = self.windows[idx]
            win.append(has_contact)
            if len(win) == self.consecutive_required and all(win):
                if self.occupancy[idx] != 1:
                    self.occupancy[idx] = 1
                    self._publish_status()
                    self._update_indicators()
            elif len(win) == self.consecutive_required and not any(win):
                if self.occupancy[idx] != 0:
                    self.occupancy[idx] = 0
                    self._publish_status()
                    self._update_indicators()
        return cb

    def _publish_status(self) -> None:
        data = list(self.occupancy)
        all_zero = all(v == 0 for v in data)
        all_one = all(v == 1 for v in data)
        if all_zero or all_one:
            self.last_status = data
            return
        self.last_status = data

        msg = UInt8MultiArray()
        msg.data = data
        self.status_pub.publish(msg)

    def _update_indicators(self, force: bool = False) -> None:
        for i, (x, y) in enumerate(self.spot_positions):
            occupied = self.occupancy[i] == 1
            green_z = self.hidden_z if occupied else self.visible_z
            red_z = self.visible_z if occupied else self.hidden_z
            if force or self.last_green_z[i] != green_z:
                self._set_model_pose(f'spot_green_0{i + 1}', x, y, green_z)
                self.last_green_z[i] = green_z
            if force or self.last_red_z[i] != red_z:
                self._set_model_pose(f'spot_red_0{i + 1}', x, y, red_z)
                self.last_red_z[i] = red_z

    def _set_model_pose(self, name: str, x: float, y: float, z: float) -> None:
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        req.pose = Pose()
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        req.pose.orientation.w = 1.0
        self.set_pose_cli.call_async(req)

    def _heartbeat(self) -> None:
        self._publish_status()

    @staticmethod
    def _contact_has_vehicle(contacts: List) -> bool:
        """Return True only if any contact involves a LIMO (dynamic or static)."""
        for c in contacts:
            names = []
            if hasattr(c, 'collision1') and hasattr(c.collision1, 'name'):
                names.append((c.collision1.name or '').lower())
            if hasattr(c, 'collision2') and hasattr(c.collision2, 'name'):
                names.append((c.collision2.name or '').lower())
            for n in names:
                if 'limo' in n:
                    return True
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingSpotIndicator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
