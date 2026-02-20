#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import cv2
import numpy as np
import socket
import yaml #
from ament_index_python.packages import get_package_share_directory
import os

# UDP listening port (must match the sender).
LISTEN_PORT = 9999

# UDP buffer size (96k is safe for 640x480 JPEG frames).
BUFFER_SIZE = 96 * 1024

class UdpCameraReceiver(Node):
    def __init__(self):
        super().__init__('udp_camera_receiver_node')
        self.get_logger().info(f"Starting UDP camera receiver on port {LISTEN_PORT}")

        # ROS publishers.
        self.pub_image = self.create_publisher(Image, '/image_raw', 10)
        self.pub_cam_info = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Load camera calibration from YAML.
        self.camera_info_msg = self._load_camera_info_from_yaml()
        if self.camera_info_msg is None:
            self.get_logger().error("Failed to load calibration; apriltag_node will not provide valid 3D poses.")
            return

        # Configure UDP socket.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Listen on all interfaces.
        self.sock.bind(("", LISTEN_PORT))
        self.sock.settimeout(1.0)

        # Timer for processing network packets.
        self.timer = self.create_timer(0.001, self.receive_and_publish)

    def _load_camera_info_from_yaml(self):
        """Load camera calibration from YAML."""
        try:
            pkg_share_dir = get_package_share_directory('limo_apriltag_tools')
            calib_file = os.path.join(pkg_share_dir, 'config', 'webcam_calibration.yaml')

            with open(calib_file, 'r') as file:
                calib_data = yaml.safe_load(file)

            cinfo = CameraInfo()
            cinfo.height = calib_data['image_height']
            cinfo.width = calib_data['image_width']
            cinfo.distortion_model = calib_data['distortion_model']
            cinfo.k = calib_data['camera_matrix']['data']
            cinfo.d = calib_data['distortion_coefficients']['data']
            cinfo.r = calib_data['rectification_matrix']['data']
            cinfo.p = calib_data['projection_matrix']['data']
            cinfo.header.frame_id = "camera"

            self.get_logger().info("Camera calibration loaded successfully.")
            return cinfo
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration YAML: {e}")
            return None

    def receive_and_publish(self):
        """Receive a UDP JPEG frame, decode, and publish ROS messages."""
        try:
            # 1) Receive JPEG data.
            data, addr = self.sock.recvfrom(BUFFER_SIZE)

            # 2) Decode JPEG to OpenCV BGR image.
            np_arr = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn("Corrupted UDP packet: could not decode JPEG.")
                return

            # 3) Convert frame to ROS Image.
            now = self.get_clock().now().to_msg()

            img_msg = Image()
            img_msg.header.stamp = now
            img_msg.header.frame_id = "camera"
            img_msg.height = frame.shape[0]
            img_msg.width = frame.shape[1]
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = False
            img_msg.step = frame.shape[1] * 3
            img_msg.data = frame.tobytes()

            # 4) Publish image.
            self.pub_image.publish(img_msg)

            # 5) Publish synchronized CameraInfo.
            if self.camera_info_msg:
                self.camera_info_msg.header.stamp = now
                self.pub_cam_info.publish(self.camera_info_msg)

        except socket.timeout:
            # Normal condition: no packet received within timeout.
            pass
        except Exception as e:
            self.get_logger().warn(f"Receiver loop error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UdpCameraReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
