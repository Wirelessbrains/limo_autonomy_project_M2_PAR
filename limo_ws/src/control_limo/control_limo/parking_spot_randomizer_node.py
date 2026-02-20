import math
import random
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class ParkingSpotRandomizer(Node):
    """Move static LIMO models to random free spots at startup."""

    def __init__(self) -> None:
        super().__init__('parking_spot_randomizer')
        self.model_names = self.declare_parameter(
            'model_names',
            ['limo_park_01', 'limo_park_03', 'limo_park_04', 'limo_park_06']
        ).value
        self.spot_targets: List[Tuple[float, float, float]] = [
            (-2.0, 1.9, -1.5708),
            (0.0, 1.9, -1.5708),
            (2.0, 1.9, -1.5708),
            (-2.0, -1.9, 1.5708),
            (0.0, -1.9, 1.5708),
            (2.0, -1.9, 1.5708),
        ]
        seed = self.declare_parameter('seed', 0).value
        if seed:
            random.seed(seed)

        self.set_pose_service = self.declare_parameter(
            'set_pose_service', '/world/limo_parking_lot/set_pose'
        ).value
        self.cli = self.create_client(SetEntityPose, self.set_pose_service)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'Service {self.set_pose_service} unavailable. Aborting randomization.'
            )
            rclpy.shutdown()
            return

        self.assign_and_move()
        self.get_logger().info('Randomization completed.')
        rclpy.shutdown()

    def assign_and_move(self) -> None:
        chosen_spots = random.sample(self.spot_targets, k=min(len(self.model_names), len(self.spot_targets)))
        for model, (x, y, yaw) in zip(self.model_names, chosen_spots):
            self._set_pose(model, x, y, 0.145, yaw)
            self.get_logger().info(f'{model} -> ({x:.2f}, {y:.2f}, yaw {yaw:.2f})')

    def _set_pose(self, model: str, x: float, y: float, z: float, yaw: float) -> None:
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = model
        req.entity.type = Entity.MODEL
        req.pose = Pose()
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        half = yaw * 0.5
        req.pose.orientation.z = math.sin(half)
        req.pose.orientation.w = math.cos(half)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    ParkingSpotRandomizer()


if __name__ == '__main__':
    main()
