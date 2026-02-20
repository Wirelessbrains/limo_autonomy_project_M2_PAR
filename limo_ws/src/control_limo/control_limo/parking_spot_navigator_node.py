import math
from typing import Any, List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray


def make_pose(x: float, y: float, yaw: float, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    half = yaw * 0.5
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)
    return msg


class ParkingSpotNavigator(Node):
    def __init__(self) -> None:
        super().__init__('parking_spot_navigator')

        self.declare_parameter('pose_topic', '/tag_only_pose')
        self.declare_parameter('spot_status_topic', '/parking/spot_status')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('reach_tolerance', 0.25)
        self.declare_parameter('park_lock_distance', 0.5)
        self.declare_parameter('entry_point', [-3.0, 0.0])
        self.declare_parameter('lane_xs', [-2.0, 0.0, 2.0])
        self.declare_parameter('lane_y', 0.0)
        self.declare_parameter('align_to_corridor', True)
        self.declare_parameter('pose_msg_type', 'pose_stamped')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_spots_key', 'parking_spots')
        self.declare_parameter('parking_entry_y_offset', 0.0)
        self.declare_parameter('decision_wp_offset_03', 0.0)
        self.declare_parameter('spot_targets', [
            -2.0, 1.9, -1.5708,
            0.0, 1.9, -1.5708,
            2.0, 1.9, -1.5708,
            -2.0, -1.9, 1.5708,
            0.0, -1.9, 1.5708,
            2.0, -1.9, 1.5708,
            -2.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            2.0, 0.0, 0.0,
        ])
        self.declare_parameter('target_spot_index', -1)
        self.declare_parameter('publish_rate', 5.0)

        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.pose_msg_type = self.get_parameter('pose_msg_type').get_parameter_value().string_value.lower()
        self.spot_status_topic = self.get_parameter('spot_status_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.reach_tolerance = self.get_parameter('reach_tolerance').get_parameter_value().double_value
        self.park_lock_distance = self.get_parameter('park_lock_distance').get_parameter_value().double_value
        entry = self.get_parameter('entry_point').get_parameter_value().double_array_value
        self.entry_point = (entry[0], entry[1])
        self.lane_xs = list(self.get_parameter('lane_xs').get_parameter_value().double_array_value)
        self.lane_y = self.get_parameter('lane_y').get_parameter_value().double_value
        self.align_to_corridor = self.get_parameter('align_to_corridor').get_parameter_value().bool_value
        self.map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        self.map_spots_key = self.get_parameter('map_spots_key').get_parameter_value().string_value
        self.parking_entry_y_offset = self.get_parameter('parking_entry_y_offset').get_parameter_value().double_value
        self.decision_wp_offset_03 = self.get_parameter('decision_wp_offset_03').get_parameter_value().double_value
        spot_targets = list(self.get_parameter('spot_targets').get_parameter_value().double_array_value)
        if len(spot_targets) % 3 != 0:
            raise ValueError('spot_targets must be a flat list of x,y,yaw triples')
        self.spot_targets = [
            (spot_targets[i], spot_targets[i + 1], spot_targets[i + 2])
            for i in range(0, len(spot_targets), 3)
        ]
        yaml_targets = self._load_spot_targets_from_yaml(self.map_yaml, self.map_spots_key)
        if yaml_targets:
            self.spot_targets = yaml_targets
            self.get_logger().info(
                f'Using {len(self.spot_targets)} spots from "{self.map_yaml}" (key "{self.map_spots_key}").'
            )
        elif self.map_yaml:
            self.get_logger().warn(
                f'No spots found in "{self.map_yaml}" under key "{self.map_spots_key}". '
                'Using spot_targets from parameters.'
            )
        self.target_spot_index = int(self.get_parameter('target_spot_index').get_parameter_value().integer_value)
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.current_pose: Optional[PoseStamped] = None
        self.spot_status: List[int] = []
        self.active_target: Optional[int] = None
        self.active_path: List[PoseStamped] = []
        self.stage = 0
        self.parked = False

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        if self.pose_msg_type == 'pose':
            self.pose_sub = self.create_subscription(Pose, self.pose_topic, self.pose_pose_cb, 10)
        elif self.pose_msg_type == 'odom':
            self.pose_sub = self.create_subscription(Odometry, self.pose_topic, self.odom_cb, 10)
        else:
            self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
        self.status_sub = self.create_subscription(UInt8MultiArray, self.spot_status_topic, self.status_cb, 10)

        self.force_publish = True

        period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(period, self.publish_goal)
        self.get_logger().info('ParkingSpotNavigator ready.')

    @staticmethod
    def _yaw_from_quat(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _parse_spot_entry(self, entry: Any) -> Optional[Tuple[float, float, float]]:
        if not isinstance(entry, dict):
            return None
        if 'x' in entry and 'y' in entry and 'yaw' in entry:
            return float(entry['x']), float(entry['y']), float(entry['yaw'])
        if 'position' in entry:
            p = entry['position']
            x = float(p['x'])
            y = float(p['y'])
            if 'yaw' in entry:
                return x, y, float(entry['yaw'])
            if 'orientation' in entry:
                o = entry['orientation']
                z = float(o.get('z', 0.0))
                w = float(o.get('w', 1.0))
                return x, y, self._yaw_from_quat(z, w)
        return None

    def _load_spot_targets_from_yaml(self, yaml_path: str, key: str) -> Optional[List[Tuple[float, float, float]]]:
        if not yaml_path:
            return None
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f'Failed to open spots YAML "{yaml_path}": {exc}')
            return None

        raw = data.get(key)
        if raw is None:
            return None

        spots: List[Tuple[float, float, float]] = []
        if isinstance(raw, list):
            if raw and all(isinstance(v, (int, float)) for v in raw):
                if len(raw) % 3 != 0:
                    self.get_logger().warn(f'List "{key}" must contain multiples of 3 (x,y,yaw).')
                    return None
                for i in range(0, len(raw), 3):
                    spots.append((float(raw[i]), float(raw[i + 1]), float(raw[i + 2])))
                return spots
            for item in raw:
                parsed = self._parse_spot_entry(item)
                if parsed is not None:
                    spots.append(parsed)
        elif isinstance(raw, dict):
            for _, item in raw.items():
                parsed = self._parse_spot_entry(item)
                if parsed is not None:
                    spots.append(parsed)

        return spots or None

    def pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def pose_pose_cb(self, msg: Pose) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose = msg
        self.current_pose = pose

    def odom_cb(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        self.current_pose = pose

    def status_cb(self, msg: UInt8MultiArray) -> None:
        self.spot_status = list(msg.data)

    def _distance_to(self, x: float, y: float) -> Optional[float]:
        if self.current_pose is None:
            return None
        dx = x - self.current_pose.pose.position.x
        dy = y - self.current_pose.pose.position.y
        return math.hypot(dx, dy)

    def _select_target_spot(self) -> Optional[int]:
        if self.target_spot_index >= 0:
            return self.target_spot_index
        priority = [0, 3, 1, 4, 2, 5]
        for idx in priority:
            if idx < len(self.spot_status) and self.spot_status[idx] == 0:
                return idx
        return None

    def _build_goal_for_stage(self, target_idx: int) -> PoseStamped:
        if not self.active_path:
            self.active_path = self._build_path_for_target(target_idx)
        return self.active_path[min(self.stage, len(self.active_path) - 1)]

    def _build_path_for_target(self, target_idx: int) -> List[PoseStamped]:
        target_x, target_y, target_yaw = self.spot_targets[target_idx]
        if not self.align_to_corridor:
            return [make_pose(target_x, target_y, target_yaw, self.frame_id)]

        start_x = self.entry_point[0]
        if self.current_pose:
            curr_x = self.current_pose.pose.position.x
            closest_lane_x = min(self.lane_xs + [self.entry_point[0]], key=lambda x: abs(x - curr_x))
            start_x = closest_lane_x

        def _between(a: float, b: float, v: float) -> bool:
            low, high = (a, b) if a <= b else (b, a)
            return low - 0.01 <= v <= high + 0.01

        path: List[PoseStamped] = []
        path.append(make_pose(start_x, 0.0, 0.0, self.frame_id))
        final_waypoint_x = target_x
        direction = 1 if final_waypoint_x >= start_x else -1

        if target_idx in (0, 3) and self.decision_wp_offset_03 > 0.0:
            final_waypoint_x += direction * self.decision_wp_offset_03

        lane_xs = sorted(self.lane_xs, reverse=direction < 0)

        for lane_x in lane_xs:
            if _between(start_x, final_waypoint_x, lane_x):
                if abs(lane_x - start_x) > 0.01 and abs(lane_x - final_waypoint_x) > 0.01:
                    path.append(make_pose(lane_x, self.lane_y, 0.0, self.frame_id))

        if abs(final_waypoint_x - start_x) > 0.01:
            path.append(make_pose(final_waypoint_x, 0.0, 0.0, self.frame_id))
        elif not path:
            path.append(make_pose(final_waypoint_x, 0.0, 0.0, self.frame_id))

        if abs(target_y) > 0.01:
            final_y = target_y
            if self.parking_entry_y_offset > 0.0:
                final_y += math.copysign(self.parking_entry_y_offset, target_y)
            path.append(make_pose(target_x, final_y, target_yaw, self.frame_id))

        return path

    def _final_target_xy(self, target_idx: int) -> Tuple[float, float]:
        target_x, target_y, _ = self.spot_targets[target_idx]
        final_y = target_y
        if self.parking_entry_y_offset > 0.0 and abs(target_y) > 0.01:
            final_y += math.copysign(self.parking_entry_y_offset, target_y)
        return target_x, final_y

    def _advance_stage_if_reached(self, goal: PoseStamped) -> None:
        dist = self._distance_to(goal.pose.position.x, goal.pose.position.y)
        if dist is None:
            return
        if dist <= self.reach_tolerance and self.stage < len(self.active_path) - 1:
            self.stage += 1
            self.force_publish = True
            self.get_logger().info(f'Advancing to stage {self.stage}')

    def _maybe_lock_parked(self, target_idx: int) -> None:
        if not self.spot_status or target_idx >= len(self.spot_status):
            return
        if self.spot_status[target_idx] == 0:
            return
        target_x, target_y = self._final_target_xy(target_idx)
        dist = self._distance_to(target_x, target_y)
        if dist is not None and dist <= self.park_lock_distance:
            self.parked = True
            self.get_logger().info('Spot occupied by robot. Parking completed.')

    def publish_goal(self) -> None:
        if self.current_pose is None:
            return

        if self.active_target is None:
            target_idx = self._select_target_spot()
            if target_idx is None:
                self.get_logger().warn('No free parking spot found.', throttle_duration_sec=5.0)
                return
        else:
            target_idx = self.active_target

        if self.active_target is None:
            self.active_target = target_idx
            self.stage = 0
            self.active_path = self._build_path_for_target(self.active_target)
            self.force_publish = True

        goal = self._build_goal_for_stage(self.active_target)
        self._advance_stage_if_reached(goal)
        goal = self._build_goal_for_stage(self.active_target)

        if self.active_path:
            final_goal = self.active_path[-1]
            final_dist = self._distance_to(final_goal.pose.position.x, final_goal.pose.position.y)
            if final_dist is not None:
                if final_dist <= max(self.reach_tolerance * 1.2, 0.30):
                    self.stage = len(self.active_path) - 1
                    goal = final_goal
                    self.force_publish = True
                if final_dist <= self.reach_tolerance:
                    self.parked = True
                    goal = final_goal
                    self.force_publish = True

        self._maybe_lock_parked(self.active_target)

        if self.parked:
            if self.force_publish:
                now = self.get_clock().now()
                goal.header.stamp = now.to_msg()
                self.goal_pub.publish(goal)
                self.force_publish = False
            return

        now = self.get_clock().now()
        if self.force_publish:
            goal.header.stamp = now.to_msg()
            self.goal_pub.publish(goal)
            self.force_publish = False



def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingSpotNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
