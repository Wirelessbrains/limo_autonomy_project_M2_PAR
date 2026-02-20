import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2

from rclpy.duration import Duration


def project_points(points_cam: np.ndarray, k: np.ndarray):
    """Project Nx3 points in camera frame to pixel coords using intrinsics matrix K."""
    zs = points_cam[:, 2]
    valid = zs > 1e-6
    uv = np.zeros((points_cam.shape[0], 2), dtype=float)
    uv[valid, 0] = (points_cam[valid, 0] * k[0, 0] / zs[valid]) + k[0, 2]
    uv[valid, 1] = (points_cam[valid, 1] * k[1, 1] / zs[valid]) + k[1, 2]
    return uv, valid


def make_axis_points(tag_size: float):
    """Return 4x3 points for origin, x, y, z axes in tag frame."""
    half = tag_size * 0.5
    return np.array(
        [
            [0.0, 0.0, 0.0],       # origin
            [half, 0.0, 0.0],      # X
            [0.0, half, 0.0],      # Y
            [0.0, 0.0, -half],     # Z (out of tag plane)
        ],
        dtype=float,
    )


def pose_to_matrix(pose_msg) -> np.ndarray:
    q = np.array(
        [
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ],
        dtype=float,
    )
    t = np.array(
        [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z],
        dtype=float,
    )
    mat = quaternion_matrix(q)
    mat[0:3, 3] = t
    return mat


class TagOverlayNode(Node):
    def __init__(self):
        super().__init__('tag_overlay_node')

        # Parameters
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('tag_size', 0.1)
        self.declare_parameter('output_topic', '/tag_overlay/image')
        self.declare_parameter('min_margin', 0.0)
        self.declare_parameter('use_tf', False)

        self.detections_topic = self.get_parameter('detections_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_size = self.get_parameter('tag_size').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_margin = self.get_parameter('min_margin').value
        self.use_tf = self.get_parameter('use_tf').value

        # QoS: image and detection streams are sensor-data (best effort).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        overlay_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge()
        self.last_image_msg = None
        self.last_cv_image = None
        self.last_image_time = None
        self.last_detections = None
        self.k_matrix = None
        self._last_caminfo_warn = 0.0

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            sensor_qos,
        )

        self.cinfo_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            sensor_qos,
        )

        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            sensor_qos,
        )

        self.pub_overlay = self.create_publisher(Image, self.output_topic, overlay_qos)

        # TF buffer to query camera->tag transform (optional)
        if self.use_tf:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        self.timer = self.create_timer(0.05, self._process)

        self.get_logger().info(
            f"Tag overlay ready. image={self.image_topic}, detections={self.detections_topic}, "
            f"output={self.output_topic}, camera_frame={self.camera_frame}, tag_size={self.tag_size}"
        )

    # Callbacks
    def _on_camera_info(self, msg: CameraInfo):
        try:
            self.k_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
        except Exception as exc:
            self.get_logger().warn(f"Error reading camera_info: {exc}")

    def _on_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f"Error converting image: {exc}")
            return
        self.last_image_msg = msg
        self.last_cv_image = cv_img
        self.last_image_time = self.get_clock().now()

    def _on_detections(self, msg: AprilTagDetectionArray):
        self.last_detections = msg

    # Main processing
    def _process(self):
        if self.last_cv_image is None or self.last_detections is None:
            return

        img = self.last_cv_image.copy()
        detections = self.last_detections.detections
        if not detections:
            return

        axes_pts_tag = make_axis_points(self.tag_size)
        overlay_lines = []
        for det in detections:
            margin = getattr(det, 'decision_margin', 0.0)
            if margin < self.min_margin:
                continue
            tag_name = self._tag_name(det)

            # Read camera->tag transform from detection pose first.
            t_cam_tag = self._pose_from_detection(det)
            if t_cam_tag is None and self.use_tf:
                t_cam_tag = self._lookup_tf(tag_name)
            if t_cam_tag is not None and self.k_matrix is not None:
                pts_cam = self._transform_points(axes_pts_tag, t_cam_tag)
                pixels, valid = project_points(pts_cam, self.k_matrix)
                if np.all(valid):
                    o = tuple(pixels[0].astype(int))
                    px = tuple(pixels[1].astype(int))
                    py = tuple(pixels[2].astype(int))
                    pz = tuple(pixels[3].astype(int))

                    cv2.line(img, o, px, (0, 0, 255), 2)   # X em vermelho
                    cv2.line(img, o, py, (0, 255, 0), 2)   # Y em verde
                    cv2.line(img, o, pz, (255, 0, 0), 2)   # Z em azul

            # Euclidean camera->tag distance from pose estimate.
            dist = None
            if t_cam_tag is not None:
                dist = float(np.linalg.norm(t_cam_tag[0:3, 3]))
            if dist is None and self.k_matrix is not None:
                try:
                    corners_px = [(float(c.x), float(c.y)) for c in det.corners]
                except Exception:
                    corners_px = []
                if len(corners_px) == 4:
                    # Approximate distance from apparent tag size in pixels.
                    edges = [
                        np.linalg.norm(np.array(corners_px[0]) - np.array(corners_px[1])),
                        np.linalg.norm(np.array(corners_px[1]) - np.array(corners_px[2])),
                        np.linalg.norm(np.array(corners_px[2]) - np.array(corners_px[3])),
                        np.linalg.norm(np.array(corners_px[3]) - np.array(corners_px[0])),
                    ]
                    size_px = float(np.mean(edges))
                    if size_px > 1e-6:
                        fx = float(self.k_matrix[0, 0])
                        dist = fx * float(self.tag_size) / size_px

            # Draw tag contour and corner indices (when corners are available).
            try:
                corners_px = [(int(c.x), int(c.y)) for c in det.corners]
                if len(corners_px) == 4:
                    cv2.polylines(img, [np.array(corners_px, dtype=np.int32)], isClosed=True, color=(0, 255, 255), thickness=2)
                    for idx, pt in enumerate(corners_px):
                        cv2.circle(img, pt, 3, (0, 165, 255), -1)
                        cv2.putText(img, str(idx), (pt[0] + 3, pt[1] - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1, cv2.LINE_AA)
            except Exception:
                pass

            line = f"{tag_name}"
            if dist is not None:
                line += f" dist={dist:.2f}m"
            line += f" margin={margin:.1f}"
            overlay_lines.append(line)

        if overlay_lines:
            h, w = img.shape[:2]
            y = h - 10 - (len(overlay_lines) - 1) * 18
            for line in overlay_lines:
                cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                y += 18

        # Publish overlay
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        if self.last_image_msg:
            out_msg.header = self.last_image_msg.header
        self.pub_overlay.publish(out_msg)

    # Helpers
    def _tag_name(self, detection):
        try:
            tag_id = detection.id[0] if isinstance(detection.id, (list, tuple)) else detection.id
        except Exception:
            tag_id = detection.id if hasattr(detection, 'id') else '0'
        family = getattr(detection, 'family', 'tag36h11')
        return f"{family}:{tag_id}"

    def _lookup_tf(self, tag_name: str):
        if self.tf_buffer is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame,
                tag_name,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            return tf
        except Exception:
            return None

    def _pose_from_detection(self, detection):
        try:
            pose_msg = detection.pose.pose.pose
        except Exception:
            return None
        return pose_to_matrix(pose_msg)

    def _transform_points(self, pts_tag: np.ndarray, tf_msg):
        """Transform Nx3 points from tag frame to camera frame using TF."""
        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        q = np.array([rot.x, rot.y, rot.z, rot.w], dtype=float)
        t = np.array([trans.x, trans.y, trans.z], dtype=float)

        # Rotation matrix from quaternion
        r = quaternion_matrix(q)[0:3, 0:3]
        return (pts_tag @ r.T) + t


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Return homogeneous rotation matrix from quaternion [x, y, z, w]."""
    q = np.array(quaternion, dtype=float, copy=True)
    n = np.dot(q, q)
    if n < np.finfo(float).eps * 4.0:
        return np.identity(4)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0.0],
        [q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0.0],
        [q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def main(args=None):
    rclpy.init(args=args)
    node = TagOverlayNode()
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
