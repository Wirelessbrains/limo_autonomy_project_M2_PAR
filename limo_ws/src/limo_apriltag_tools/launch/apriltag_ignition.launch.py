import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_name = 'limo_apriltag_tools'
    pkg_share_dir = get_package_share_directory(pkg_name)

    apriltag_params_file = os.path.join(
        pkg_share_dir,
        'config',
        'apriltag_params.yaml'
    )

    image_topic = LaunchConfiguration('image_topic')
    camera_image_topic = LaunchConfiguration('camera_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    tag_size = LaunchConfiguration('tag_size')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_camera_info_publisher = LaunchConfiguration('enable_camera_info_publisher')
    calibration_file = LaunchConfiguration('calibration_file')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        remappings=[
            ('image_rect', image_topic),
            ('camera_info', camera_info_topic),
        ],
        parameters=[
            apriltag_params_file,
            {'use_sim_time': use_sim_time},
            {'size': tag_size},
        ],
    )

    camera_info_publisher_node = Node(
        package='limo_apriltag_tools',
        executable='camera_info_publisher_node',
        name='camera_info_publisher',
        output='screen',
        parameters=[
            {'calibration_file': calibration_file},
            {'image_topic': camera_image_topic},
            {'camera_info_topic': camera_info_topic},
            {'image_output_topic': image_topic},
            {'relay_image': True},
        ],
        condition=IfCondition(enable_camera_info_publisher),
    )

    overlay_node = Node(
        package='limo_apriltag_tools',
        executable='tag_overlay_node',
        name='tag_overlay_node',
        output='screen',
        parameters=[
            {'camera_frame': camera_frame},
            {'detections_topic': '/detections'},
            {'image_topic': image_topic},
            {'camera_info_topic': camera_info_topic},
            {'tag_size': tag_size},
            {'output_topic': '/tag_overlay/image'},
            {'min_margin': 20.0},
            {'use_tf': False},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_file',
            default_value=os.path.join(pkg_share_dir, 'config', 'ign_camera_info.yaml'),
            description='Camera calibration YAML for simulated RGB camera',
        ),
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value='/rgb_camera',
            description='Ignition RGB camera topic (input)',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/rgb_camera_sync',
            description='Image topic used by AprilTag detector',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='Camera info topic used by the detector/overlay',
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='limo_01/depth_camera_link/rgb_camera',
            description='Camera frame id used by the tag overlay node',
        ),
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.16',
            description='AprilTag size (meters)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'enable_camera_info_publisher',
            default_value='true',
            description='Publish CameraInfo from YAML for the simulated camera',
        ),
        camera_info_publisher_node,
        apriltag_node,
        overlay_node,
    ])
