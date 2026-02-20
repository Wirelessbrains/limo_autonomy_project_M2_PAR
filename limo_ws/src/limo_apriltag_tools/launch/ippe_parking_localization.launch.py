import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'limo_apriltag_tools'
    pkg_share_dir = get_package_share_directory(pkg_name)

    # Arguments
    detections_topic = LaunchConfiguration('detections_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_size = LaunchConfiguration('tag_size')
    map_yaml = LaunchConfiguration('map_yaml')
    base_frame = LaunchConfiguration('base_frame')
    use_tf_tag_estimates = LaunchConfiguration('use_tf_tag_estimates')
    use_detections_topic = LaunchConfiguration('use_detections_topic')
    tf_weight = LaunchConfiguration('tf_weight')
    ippe_weight = LaunchConfiguration('ippe_weight')
    update_rate = LaunchConfiguration('update_rate')
    single_tag_alpha = LaunchConfiguration('single_tag_alpha')
    base_smooth_alpha = LaunchConfiguration('base_smooth_alpha')

    return LaunchDescription([
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/detections',
            description='Topic with AprilTag detections'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.16',
            description='Size of the AprilTags in meters'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg_share_dir, 'config', 'tag_map_parking.yaml'),
            description='Path to the tag map YAML file'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base frame to publish (base_footprint for ground, base_link for body center)'
        ),
        DeclareLaunchArgument('use_tf_tag_estimates', default_value='false',
                              description='Deprecated in IPPE-only mode (kept for compatibility)'),
        DeclareLaunchArgument('use_detections_topic', default_value='true',
                              description='Use /detections corners (IPPE-only mode)'),
        DeclareLaunchArgument('tf_weight', default_value='1.5',
                              description='Weight for TF tag estimates in fusion'),
        DeclareLaunchArgument('ippe_weight', default_value='1.0',
                              description='Weight for IPPE estimates in fusion'),
        DeclareLaunchArgument('update_rate', default_value='20.0',
                              description='Estimator loop rate in Hz (for TF-only updates too)'),
        DeclareLaunchArgument('single_tag_alpha', default_value='0.15',
                              description='Smoothing alpha when only one estimate is available'),
        DeclareLaunchArgument('pnp_method', default_value='iterative',
                              description="Per-tag pose method: 'iterative' (offline-like) or 'ippe'"),
        DeclareLaunchArgument('apply_rx180_correction', default_value='false'),
        DeclareLaunchArgument('enforce_camera_in_front_of_tag', default_value='true'),
        DeclareLaunchArgument('camera_front_sign', default_value='-1.0'),
        DeclareLaunchArgument('base_smooth_alpha', default_value='0.08',
                              description='Planar smoothing alpha for base footprint output'),

        # 1. Static Map Publisher (visualizes tags in RViz)
        Node(
            package=pkg_name,
            executable='static_map_publisher',
            name='static_map_publisher',
            output='screen',
            parameters=[{'map_yaml': map_yaml}]
        ),

        # 2. IPPE Localizer (computes robot pose)
        Node(
            package=pkg_name,
            executable='ippe_localizer',
            name='ippe_localizer',
            output='screen',
            parameters=[{
                'detections_topic': detections_topic,
                'camera_info_topic': camera_info_topic,
                'tag_size': tag_size,
                'map_yaml': map_yaml,
                'base_frame': base_frame,
                'use_tf_tag_estimates': use_tf_tag_estimates,
                'use_detections_topic': use_detections_topic,
                'tf_weight': tf_weight,
                'ippe_weight': ippe_weight,
                'update_rate': update_rate,
                'single_tag_alpha': single_tag_alpha,
                'pnp_method': LaunchConfiguration('pnp_method'),
                'apply_rx180_correction': LaunchConfiguration('apply_rx180_correction'),
                'enforce_camera_in_front_of_tag': LaunchConfiguration('enforce_camera_in_front_of_tag'),
                'camera_front_sign': LaunchConfiguration('camera_front_sign'),
                'base_smooth_alpha': base_smooth_alpha
            }]
        ),
    ])
