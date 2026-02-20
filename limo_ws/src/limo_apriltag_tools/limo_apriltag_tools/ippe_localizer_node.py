import os
import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Vector3, Point
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray

import tf2_ros
from tf2_ros import TransformBroadcaster


def quaternion_matrix(quaternion):
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
        [0.0, 0.0, 0.0, 1.0]
    ])

def quaternion_from_matrix(matrix):
    q = np.empty((4, ), dtype=float)
    M = np.array(matrix, dtype=float, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - M[j, j] - M[k, k] + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / np.sqrt(t * M[3, 3])
    return q


def pose_to_matrix(pose):
    T = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def weighted_quaternion_average(quats, weights):
    q_acc = np.zeros(4, dtype=float)
    q_ref = quats[0]
    for q, w in zip(quats, weights):
        qq = q.copy()
        if np.dot(q_ref, qq) < 0.0:
            qq = -qq
        q_acc += float(w) * qq
    n = np.linalg.norm(q_acc)
    if n < 1e-9:
        return q_ref
    return q_acc / n

class IPPELocalizerNode(Node):
    def __init__(self):
        super().__init__('ippe_localizer_node')

        # --- Parameters ---
        self.declare_parameter('tag_size', 0.16)
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('apply_rx180_correction', False)
        self.declare_parameter('debug_pose_metrics', False)
        self.declare_parameter('smoothing_alpha', 0.5)
        self.declare_parameter('single_tag_alpha', 0.12)
        self.declare_parameter('enforce_camera_in_front_of_tag', True)
        self.declare_parameter('camera_front_sign', -1.0)
        self.declare_parameter('front_check_margin_m', 0.01)
        self.declare_parameter('temporal_consistency_weight', 0.2)
        self.declare_parameter('temporal_angle_weight', 0.8)
        self.declare_parameter('pnp_method', 'iterative')
        self.declare_parameter('use_detector_pose', True)
        self.declare_parameter('enable_global_refine', True)
        self.declare_parameter('refine_min_tags', 2)
        self.declare_parameter('enable_refine_ransac', True)
        self.declare_parameter('ransac_reproj_error_px', 3.0)
        self.declare_parameter('ransac_iterations', 100)
        self.declare_parameter('ransac_confidence', 0.99)
        self.declare_parameter('max_refine_rms_px', 4.0)
        self.declare_parameter('enable_tag_bias_correction', True)
        self.declare_parameter('bias_correction_min_tags', 2)
        self.declare_parameter('bias_correction_max_m', 0.60)

        self.tag_size = self.get_parameter('tag_size').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.apply_rx180_correction = self.get_parameter('apply_rx180_correction').value
        self.debug_pose_metrics = self.get_parameter('debug_pose_metrics').value
        self.alpha = self.get_parameter('smoothing_alpha').value
        self.single_tag_alpha = self.get_parameter('single_tag_alpha').value
        self.enforce_camera_in_front_of_tag = self.get_parameter('enforce_camera_in_front_of_tag').value
        self.camera_front_sign = float(self.get_parameter('camera_front_sign').value)
        self.front_check_margin_m = float(self.get_parameter('front_check_margin_m').value)
        self.temporal_consistency_weight = float(self.get_parameter('temporal_consistency_weight').value)
        self.temporal_angle_weight = float(self.get_parameter('temporal_angle_weight').value)
        self.pnp_method = str(self.get_parameter('pnp_method').value).strip().lower()
        self.use_detector_pose = bool(self.get_parameter('use_detector_pose').value)
        self.enable_global_refine = self.get_parameter('enable_global_refine').value
        self.refine_min_tags = self.get_parameter('refine_min_tags').value
        self.enable_refine_ransac = self.get_parameter('enable_refine_ransac').value
        self.ransac_reproj_error_px = self.get_parameter('ransac_reproj_error_px').value
        self.ransac_iterations = self.get_parameter('ransac_iterations').value
        self.ransac_confidence = self.get_parameter('ransac_confidence').value
        self.max_refine_rms_px = self.get_parameter('max_refine_rms_px').value
        self.enable_tag_bias_correction = self.get_parameter('enable_tag_bias_correction').value
        self.bias_correction_min_tags = self.get_parameter('bias_correction_min_tags').value
        self.bias_correction_max_m = self.get_parameter('bias_correction_max_m').value

        map_path = self.get_parameter('map_yaml').value
        if not map_path:
            map_path = os.path.join(get_package_share_directory('limo_apriltag_tools'), 'config', 'tag_map_parking.yaml')

        self.tag_map = self.load_tag_map(map_path)
        self.get_logger().info(f"Mode: Camera in Map. {len(self.tag_map)} tags loaded.")

        # --- 3D Object Points ---
        s = self.tag_size / 2.0
        # Match offline pipeline convention: TL, TR, BR, BL in image with
        # object points [-s,-s], [s,-s], [s,s], [-s,s].
        self.object_points = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], dtype=np.float32)

        # --- State ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_T_map_cam = None
        self.last_detection_stamp_ns = None
        self._last_refine_warn_ns = 0

        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos)
        self.create_subscription(AprilTagDetectionArray, self.detections_topic, self.detection_callback, qos)

        # Unified topics for the control stack
        self.pose_pub = self.create_publisher(PoseStamped, '/tag_only_pose', 10)
        self.base_pose_pub = self.create_publisher(PoseStamped, '/tag_only_base_pose', 10)

        self.create_timer(2.0, self._health_check)
        self.get_logger().info(
            f"Subscribed topics: camera_info={self.camera_info_topic}, detections={self.detections_topic}"
        )
        self.get_logger().info(
            f"IPPE correction: apply_rx180_correction={self.apply_rx180_correction}"
        )
        self.get_logger().info(
            f"Global refine: enabled={self.enable_global_refine}, "
            f"min_tags={self.refine_min_tags}, ransac={self.enable_refine_ransac}"
        )
        self.get_logger().info(
            f"Smoothing: multi_tag_alpha={self.alpha:.2f}, single_tag_alpha={self.single_tag_alpha:.2f}"
        )
        self.get_logger().info(
            f"Front check: enabled={self.enforce_camera_in_front_of_tag}, sign={self.camera_front_sign:+.1f}"
        )
        self.get_logger().info(f"Per-tag PnP method: {self.pnp_method}")
        self.get_logger().info(f"Use detector pose first: {self.use_detector_pose}")
        self.get_logger().info(
            f"Bias correction: enabled={self.enable_tag_bias_correction}, min_tags={self.bias_correction_min_tags}, max={self.bias_correction_max_m:.2f}m"
        )

    def load_tag_map(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        tags = {}
        for key, info in data.get('tags', {}).items():
            try:
                tag_id = int(str(key).split(':')[-1])
                p, o = info['position'], info['orientation']
                T = quaternion_matrix([o['x'], o['y'], o['z'], o['w']])
                T[0:3, 3] = [p['x'], p['y'], p['z']]
                tags[tag_id] = T
            except:
                continue
        return tags

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def detection_callback(self, msg):
        self.last_detection_stamp_ns = self.get_clock().now().nanoseconds
        if self.camera_matrix is None or not msg.detections:
            return

        camera_frame = msg.header.frame_id
        estimates = []
        estimate_weights = []
        obj_points_map = []
        img_points_all = []
        observed_tag_ids = []
        observed_T_cam_tag = []

        for det in msg.detections:
            if det.id not in self.tag_map:
                continue

            img_pts = np.array([[c.x, c.y] for c in det.corners], dtype=np.float32)
            if img_pts.shape != (4, 2):
                continue

            best_T_cam_tag = None
            min_score = float('inf')

            # Prefer detector-provided pose to avoid square-corner ambiguity in PnP.
            if self.use_detector_pose and hasattr(det, 'pose'):
                try:
                    pose_msg = det.pose.pose.pose
                    T_det = pose_to_matrix(pose_msg)
                    if float(T_det[2, 3]) > 0.0:
                        best_T_cam_tag = T_det
                        min_score = 0.0
                except Exception:
                    pass

            rvecs, tvecs = [], []
            if best_T_cam_tag is None:
                if self.pnp_method == 'ippe':
                    try:
                        _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                            self.object_points,
                            img_pts,
                            self.camera_matrix,
                            self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE,
                        )
                    except Exception:
                        rvecs, tvecs = [], []
                else:
                    ok, rvec, tvec = cv2.solvePnP(
                        self.object_points,
                        img_pts,
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_ITERATIVE,
                    )
                    rvecs, tvecs = ([rvec], [tvec]) if ok else ([], [])

            if best_T_cam_tag is None:
                for i in range(len(rvecs)):
                    rv, tv = rvecs[i], tvecs[i]
                    if tv[2] <= 0:
                        continue

                    # 1. Refine the raw solution before any correction
                    try:
                        rv, tv = cv2.solvePnPRefineLM(
                        self.object_points, img_pts, self.camera_matrix, self.dist_coeffs, rv, tv
                    )
                    except Exception:
                        pass

                    projected, _ = cv2.projectPoints(
                    self.object_points, rv, tv, self.camera_matrix, self.dist_coeffs
                )
                    projected = projected.reshape(-1, 2)
                    reproj_px = float(np.sqrt(np.mean(np.sum((projected - img_pts) ** 2, axis=1))))

                    R, _ = cv2.Rodrigues(rv)
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = tv.flatten()

                    # 2. Apply 180-degree correction if needed to align with map convention (Z-In vs Z-Out)
                    if self.apply_rx180_correction:
                        Rx_180 = np.diag([1.0, -1.0, -1.0, 1.0])
                        T = T @ Rx_180

                    # 3. Compute front-facing metric in the final convention
                    n_cam = T[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=float)
                    tag_to_cam = -T[:3, 3]
                    tag_to_cam /= (np.linalg.norm(tag_to_cam) + 1e-6)
                    front_metric = float(np.dot(n_cam, tag_to_cam))

                    valid_front = True
                    if self.enforce_camera_in_front_of_tag:
                        if self.camera_front_sign * front_metric <= self.front_check_margin_m:
                            valid_front = False

                    # 4. Compute score (reprojection error + temporal consistency)
                    score = reproj_px
                    if not valid_front:
                        score += 1000.0  # Heavy penalty for a ghost solution behind the tag

                    if self.last_T_map_cam is not None:
                        T_map_cam_candidate = self.tag_map[det.id] @ np.linalg.inv(T)
                        dpos = np.linalg.norm(T_map_cam_candidate[:3, 3] - self.last_T_map_cam[:3, 3])
                        score += self.temporal_consistency_weight * dpos

                        # Penalize angular jumps to reduce IPPE branch switching (especially with 1 tag)
                        R_rel = self.last_T_map_cam[:3, :3].T @ T_map_cam_candidate[:3, :3]
                        tr = float(np.trace(R_rel))
                        cosang = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
                        dang = float(np.arccos(cosang))
                        score += self.temporal_angle_weight * dang

                    if score < min_score:
                        min_score = score
                        best_T_cam_tag = T

            if best_T_cam_tag is not None:
                # T_map_cam = T_map_tag * T_tag_cam
                T_map_cam = self.tag_map[det.id] @ np.linalg.inv(best_T_cam_tag)
                estimates.append(T_map_cam)
                estimate_weights.append(1.0 / (float(min_score) + 1e-6))

                # Prepare data for optional global refinement
                corners_tag_h = np.hstack((
                    self.object_points.astype(np.float64),
                    np.ones((self.object_points.shape[0], 1), dtype=np.float64),
                ))
                # If Rx180 is applied, map object points must follow the same convention
                T_map_tag_effective = self.tag_map[det.id]
                corners_map = (T_map_tag_effective @ corners_tag_h.T).T[:, :3]
                obj_points_map.append(corners_map.astype(np.float32))
                img_points_all.append(img_pts.astype(np.float32))
                observed_tag_ids.append(det.id)
                observed_T_cam_tag.append(best_T_cam_tag.copy())

        if not estimates:
            return

        # Instant fusion: robust with 1 tag, improves with N tags via weights.
        weights = np.array(estimate_weights, dtype=float)
        if np.sum(weights) <= 1e-9:
            weights = np.ones(len(estimates), dtype=float)
        weights /= np.sum(weights)

        pos_stack = np.stack([T[:3, 3] for T in estimates], axis=0)
        avg_pos = np.average(pos_stack, axis=0, weights=weights)
        quats = [quaternion_from_matrix(T) for T in estimates]
        avg_q = weighted_quaternion_average(quats, weights)

        T_measurement = quaternion_matrix(avg_q)
        T_measurement[:3, 3] = avg_pos

        min_tags_for_refine = max(2, int(self.refine_min_tags))
        if self.enable_global_refine and len(obj_points_map) >= min_tags_for_refine:
            refined = self._refine_pose_with_all_corners(
                T_measurement,
                np.vstack(obj_points_map),
                np.vstack(img_points_all),
            )
            if refined is not None:
                T_measurement = refined
        if self.enable_tag_bias_correction and len(observed_tag_ids) >= int(self.bias_correction_min_tags):
            bias = self._compute_tag_translation_bias(T_measurement, observed_tag_ids, observed_T_cam_tag)
            if bias is not None:
                T_measurement[:3, 3] += bias
                if self.debug_pose_metrics:
                    self.get_logger().info(
                        f"bias_correction: dx={bias[0]:+.3f} dy={bias[1]:+.3f} dz={bias[2]:+.3f} m"
                    )

        if self.debug_pose_metrics and obj_points_map:
            rms_px = self._compute_reprojection_rms(
                T_measurement,
                np.vstack(obj_points_map),
                np.vstack(img_points_all),
            )
            self.get_logger().info(
                f"pose_metrics: tags={len(obj_points_map)} z={T_measurement[2,3]:.3f}m rms={rms_px:.2f}px"
            )

        # Temporal smoothing
        T_final = T_measurement

        if self.last_T_map_cam is not None:
            alpha = self.single_tag_alpha if len(estimates) == 1 else self.alpha
            pos_last = self.last_T_map_cam[:3, 3]
            pos_meas = T_measurement[:3, 3]
            pos_smooth = (1 - alpha) * pos_last + alpha * pos_meas

            q_last = quaternion_from_matrix(self.last_T_map_cam)
            q_meas = quaternion_from_matrix(T_measurement)
            if np.dot(q_last, q_meas) < 0:
                q_meas = -q_meas
            q_smooth = (1 - alpha) * q_last + alpha * q_meas
            q_smooth /= np.linalg.norm(q_smooth)

            T_final = quaternion_matrix(q_smooth)
            T_final[:3, 3] = pos_smooth

        self.last_T_map_cam = T_final
        self.publish(T_final, camera_frame, msg.header.stamp)

    def _compute_tag_translation_bias(self, T_map_cam, tag_ids, T_cam_tags):
        residuals = []
        for tag_id, T_cam_tag in zip(tag_ids, T_cam_tags):
            T_ref = self.tag_map.get(tag_id)
            if T_ref is None:
                continue
            T_pred = T_map_cam @ T_cam_tag
            residuals.append(T_ref[:3, 3] - T_pred[:3, 3])

        if not residuals:
            return None

        bias = np.mean(np.stack(residuals, axis=0), axis=0)
        max_norm = float(self.bias_correction_max_m)
        n = float(np.linalg.norm(bias))
        if max_norm > 0.0 and n > max_norm:
            bias = bias * (max_norm / n)
        return bias

    def _refine_pose_with_all_corners(self, T_map_cam_init, object_points_map, image_points):
        if object_points_map.shape[0] < 4:
            return None

        try:
            T_cam_map_init = np.linalg.inv(T_map_cam_init)
        except np.linalg.LinAlgError:
            return None

        rvec_init, _ = cv2.Rodrigues(T_cam_map_init[:3, :3])
        tvec_init = T_cam_map_init[:3, 3].reshape(3, 1)

        obj = object_points_map.astype(np.float32)
        img = image_points.astype(np.float32)
        rvec = rvec_init.astype(np.float64)
        tvec = tvec_init.astype(np.float64)

        if self.enable_refine_ransac:
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj,
                img,
                self.camera_matrix,
                self.dist_coeffs,
                rvec=rvec,
                tvec=tvec,
                useExtrinsicGuess=True,
                iterationsCount=int(self.ransac_iterations),
                reprojectionError=float(self.ransac_reproj_error_px),
                confidence=float(self.ransac_confidence),
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                return None
            if inliers is not None and len(inliers) >= 4:
                idx = inliers[:, 0]
                obj = obj[idx]
                img = img[idx]

        rvec, tvec = cv2.solvePnPRefineLM(
            obj,
            img,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
        )

        projected, _ = cv2.projectPoints(obj, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        projected = projected.reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected - img) ** 2, axis=1))))
        if rms > float(self.max_refine_rms_px):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_refine_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"Global refinement rejected: RMS={rms:.2f}px "
                    f"(threshold={float(self.max_refine_rms_px):.2f}px)"
                )
                self._last_refine_warn_ns = now_ns
            return None

        R, _ = cv2.Rodrigues(rvec)
        T_cam_map = np.eye(4)
        T_cam_map[:3, :3] = R
        T_cam_map[:3, 3] = tvec.flatten()
        try:
            return np.linalg.inv(T_cam_map)
        except np.linalg.LinAlgError:
            return None

    def _compute_reprojection_rms(self, T_map_cam, object_points_map, image_points):
        try:
            T_cam_map = np.linalg.inv(T_map_cam)
        except np.linalg.LinAlgError:
            return float('inf')
        rvec, _ = cv2.Rodrigues(T_cam_map[:3, :3])
        tvec = T_cam_map[:3, 3].reshape(3, 1)
        projected, _ = cv2.projectPoints(
            object_points_map.astype(np.float32),
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        projected = projected.reshape(-1, 2)
        img = image_points.astype(np.float32)
        return float(np.sqrt(np.mean(np.sum((projected - img) ** 2, axis=1))))

    def publish(self, T, camera_frame, stamp):
        q = quaternion_from_matrix(T)

        # 1. Publish Camera Pose (map -> camera)
        ts = TransformStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = self.map_frame
        ts.child_frame_id = camera_frame
        ts.transform.translation = Vector3(x=T[0,3], y=T[1,3], z=T[2,3])
        ts.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(ts)

        ps = PoseStamped()
        ps.header = ts.header
        ps.pose.position = Point(x=T[0,3], y=T[1,3], z=T[2,3])
        ps.pose.orientation = ts.transform.rotation
        self.pose_pub.publish(ps)

        # 2. Compute and Publish Base Pose (map -> base_footprint)
        # LIMO camera is ~0.084 m in front of center and ~0.15 m above the ground.
        # Publish a ground-projected pose (z=0) without roll/pitch.

        # Convert map->camera to map->base using optical-camera->base extrinsics
        T_cam_base = np.eye(4)
        # base->optical (xacro): R=[[0,0,1],[-1,0,0],[0,-1,0]], t=[0.084,0,0.18]
        T_base_opt = np.eye(4)
        T_base_opt[0:3, 0:3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        T_base_opt[0:3, 3] = np.array([0.084, 0.0, 0.18], dtype=float)
        T_cam_base = np.linalg.inv(T_base_opt)

        T_map_base = T @ T_cam_base

        # Planar heading from base X-axis in map frame
        x_axis_map = T_map_base[0:3, 0]
        yaw = float(np.arctan2(x_axis_map[1], x_axis_map[0]))

        base_ps = PoseStamped()
        base_ps.header.stamp = stamp
        base_ps.header.frame_id = self.map_frame
        base_ps.pose.position = Point(x=float(T_map_base[0, 3]), y=float(T_map_base[1, 3]), z=0.0)

        # Yaw-only quaternion (stability for controller input)
        base_ps.pose.orientation.z = np.sin(yaw / 2.0)
        base_ps.pose.orientation.w = np.cos(yaw / 2.0)

        self.base_pose_pub.publish(base_ps)

        # Also publish base TF for RViz
        tf_base = TransformStamped()
        tf_base.header = base_ps.header
        tf_base.child_frame_id = str(self.base_frame)
        tf_base.transform.translation.x = base_ps.pose.position.x
        tf_base.transform.translation.y = base_ps.pose.position.y
        tf_base.transform.translation.z = base_ps.pose.position.z
        tf_base.transform.rotation = base_ps.pose.orientation
        self.tf_broadcaster.sendTransform(tf_base)

    def _health_check(self):
        if self.camera_matrix is None:
            self.get_logger().warn(
                f"Waiting for CameraInfo on {self.camera_info_topic}; no intrinsics, no pose."
            )
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.last_detection_stamp_ns is None or (now_ns - self.last_detection_stamp_ns) > int(3e9):
            self.get_logger().warn(
                f"No recent detections on {self.detections_topic}; pose will not be published."
            )

def main(args=None):
    rclpy.init(args=args)
    node = IPPELocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
