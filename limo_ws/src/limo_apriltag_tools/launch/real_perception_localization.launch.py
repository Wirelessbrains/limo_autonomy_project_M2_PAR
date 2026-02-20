import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('limo_apriltag_tools')

    camera_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'apriltag_camera_pipeline.launch.py')
        ),
        launch_arguments={
            'calibration_file': LaunchConfiguration('calibration_file'),
            'camera_image_topic': LaunchConfiguration('camera_image_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'tag_size': LaunchConfiguration('tag_size'),
            'use_sim_time': 'false',
            'enable_camera_info_publisher': LaunchConfiguration('enable_camera_info_publisher'),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ippe_parking_localization.launch.py')
        ),
        launch_arguments={
            'detections_topic': '/detections',
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'tag_size': LaunchConfiguration('tag_size'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            'base_frame': LaunchConfiguration('base_frame'),
            'use_detections_topic': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg_share, 'config', 'tag_map_parking.yaml'),
            description='Tag map used for localization',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=os.path.join(pkg_share, 'config', 'webcam_calibration_robot.yaml'),
            description='Calibration file for the real camera',
        ),
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value='/image_raw',
            description='Input image topic from the real camera',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_rect',
            description='Image topic used by the detector',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='CameraInfo topic',
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera',
            description='Camera frame ID',
        ),
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.16',
            description='Tag size in meters',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base frame',
        ),
        DeclareLaunchArgument(
            'enable_camera_info_publisher',
            default_value='true',
            description='Publish CameraInfo from YAML calibration',
        ),
        camera_pipeline,
        localization,
    ])
