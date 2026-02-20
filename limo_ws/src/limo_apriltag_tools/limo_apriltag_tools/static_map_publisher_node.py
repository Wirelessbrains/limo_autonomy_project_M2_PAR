import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        
        # --- Parameters ---
        default_map = os.path.join(
            get_package_share_directory('limo_apriltag_tools'),
            'config',
            'tag_map_parking.yaml'
        )
        self.declare_parameter('map_yaml', default_map)
        self.declare_parameter('world_frame', 'map')
        # Keep map tag frames separate from detector tag frames to avoid TF conflicts
        # (camera->tag from detector and map->tag from static map cannot share child frame id).
        self.declare_parameter('tag_tf_prefix', 'map_')

        map_yaml_path = self.get_parameter('map_yaml').value
        self.world_frame = self.get_parameter('world_frame').value
        self.tag_tf_prefix = self.get_parameter('tag_tf_prefix').value

        # Use both Static and Dynamic broadcasters to be safe
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Loading map from: {map_yaml_path}")
        self.transforms = self._load_transforms(map_yaml_path)

        if self.transforms:
            # Publish once as static
            self.static_broadcaster.sendTransform(self.transforms)
            self.get_logger().info(f"Published static transforms for {len(self.transforms)} tags.")
            
            # Also publish periodically (1Hz) to ensure visibility
            self.timer = self.create_timer(1.0, self.timer_callback)
        else:
            self.get_logger().warn("No valid transforms found to publish.")

    def timer_callback(self):
        # Update timestamp and republish
        now = self.get_clock().now().to_msg()
        for t in self.transforms:
            t.header.stamp = now
        self.tf_broadcaster.sendTransform(self.transforms)

    def _load_tag_map(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"Map file not found: {path}")
            return {}
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('tags', {})
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            return {}

    def _load_transforms(self, path):
        tags = self._load_tag_map(path)
        if not tags:
            return []

        transforms = []
        for tag_id, data in tags.items():
            t = TransformStamped()
            t.header.frame_id = self.world_frame
            child_frame = f"{self.tag_tf_prefix}{tag_id}"
            t.child_frame_id = child_frame

            try:
                p = data['position']
                o = data['orientation']
                t.transform.translation = Vector3(x=float(p['x']), y=float(p['y']), z=float(p['z']))
                t.transform.rotation = Quaternion(x=float(o['x']), y=float(o['y']), z=float(o['z']), w=float(o['w']))
                transforms.append(t)
            except KeyError:
                continue
        return transforms

def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
