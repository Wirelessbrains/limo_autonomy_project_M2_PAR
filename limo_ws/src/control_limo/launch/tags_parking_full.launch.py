import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_env_share = get_package_share_directory('gz_apriltag_env')
    apriltag_share = get_package_share_directory('limo_apriltag_tools')

    default_world = os.path.join(gz_env_share, 'worlds', 'parking_6spots_limo.sdf')
    default_map_yaml = os.path.join(apriltag_share, 'config', 'tag_map_parking.yaml')
    models_path = os.path.join(gz_env_share, 'models')
    worlds_path = os.path.join(gz_env_share, 'worlds')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='SDF world file',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Tag map YAML used by static_map_publisher and IPPE localizer',
    )
    target_spot_arg = DeclareLaunchArgument(
        'target_spot_index',
        default_value='-1',
        description='Parking spot index [0..5], -1 for automatic free spot',
    )
    align_to_corridor_arg = DeclareLaunchArgument(
        'align_to_corridor',
        default_value='true',
        description='If false, sends direct goal to parking spot (no corridor alignment waypoints)',
    )
    require_final_orientation_arg = DeclareLaunchArgument(
        'require_final_orientation',
        default_value='false',
        description='Rotate to target yaw at end of parking',
    )
    reach_tolerance_arg = DeclareLaunchArgument(
        'reach_tolerance',
        default_value='0.20',
        description='Navigator positional tolerance (meters)',
    )
    distance_tolerance_arg = DeclareLaunchArgument(
        'distance_tolerance',
        default_value='0.20',
        description='Controller positional tolerance (meters)',
    )
    parking_entry_y_offset_arg = DeclareLaunchArgument(
        'parking_entry_y_offset',
        default_value='0.15',
        description='Extra Y offset for final parking target (meters)',
    )
    decision_wp_offset_03_arg = DeclareLaunchArgument(
        'decision_wp_offset_03',
        default_value='0.20',
        description='Forward offset for decision waypoint on spots 0/3 (meters)',
    )
    park_lock_distance_arg = DeclareLaunchArgument(
        'park_lock_distance',
        default_value='0.20',
        description='Distance threshold to lock parked state when spot is occupied (meters)',
    )

    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{models_path}:{worlds_path}',
    )
    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{worlds_path}',
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', LaunchConfiguration('world')],
        output='screen',
    )

    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/limo_01/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
            '/model/limo_01/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/rgb_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/limo_parking_lot/model/parking_sensors/link/spot_01_zone/sensor/spot_01_sensor/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/limo_parking_lot/model/parking_sensors/link/spot_02_zone/sensor/spot_02_sensor/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/limo_parking_lot/model/parking_sensors/link/spot_03_zone/sensor/spot_03_sensor/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/limo_parking_lot/model/parking_sensors/link/spot_04_zone/sensor/spot_04_sensor/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/limo_parking_lot/model/parking_sensors/link/spot_05_zone/sensor/spot_05_sensor/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/limo_parking_lot/model/parking_sensors/link/spot_06_zone/sensor/spot_06_sensor/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/limo_parking_lot/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
    )

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apriltag_share, 'launch', 'apriltag_ignition.launch.py')
        )
    )

    ippe_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apriltag_share, 'launch', 'ippe_parking_localization.launch.py')
        ),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
        }.items(),
    )

    randomizer = ExecuteProcess(
        cmd=['ros2', 'run', 'control_limo', 'parking_spot_randomizer_node'],
        output='screen',
    )

    go_to_pose = Node(
        package='control_limo',
        executable='go_to_pose',
        name='go_to_pose_node',
        output='screen',
        parameters=[{
            'pose_source': 'tag_only',
            'pose_topic': '/tag_only_base_pose',
            'path_frame_id': 'map',
            'kp_linear': 1.1,
            'max_linear_speed': 0.45,
            'kp_angular': 3.2,
            'max_angular_speed': 2.2,
            'require_final_orientation': LaunchConfiguration('require_final_orientation'),
            'distance_tolerance': LaunchConfiguration('distance_tolerance'),
            'startup_delay': 2.0,
            'enable_reverse_motion': True,
            'use_gazebo_model_states': False,
        }],
        remappings=[('/cmd_vel', '/model/limo_01/cmd_vel')],
    )

    navigator = Node(
        package='control_limo',
        executable='parking_spot_navigator',
        name='parking_spot_navigator',
        output='screen',
        parameters=[{
            'pose_msg_type': 'pose_stamped',
            'pose_topic': '/tag_only_base_pose',
            'frame_id': 'map',
            'map_yaml': LaunchConfiguration('map_yaml'),
            'map_spots_key': 'parking_spots',
            'target_spot_index': LaunchConfiguration('target_spot_index'),
            'align_to_corridor': LaunchConfiguration('align_to_corridor'),
            'reach_tolerance': LaunchConfiguration('reach_tolerance'),
            'parking_entry_y_offset': LaunchConfiguration('parking_entry_y_offset'),
            'decision_wp_offset_03': LaunchConfiguration('decision_wp_offset_03'),
            'park_lock_distance': LaunchConfiguration('park_lock_distance'),
        }],
    )

    indicator = Node(
        package='control_limo',
        executable='parking_spot_indicator_node',
        name='parking_spot_indicator',
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        map_yaml_arg,
        target_spot_arg,
        align_to_corridor_arg,
        require_final_orientation_arg,
        reach_tolerance_arg,
        distance_tolerance_arg,
        parking_entry_y_offset_arg,
        decision_wp_offset_03_arg,
        park_lock_distance_arg,
        set_ign_resource,
        set_gz_resource,
        gazebo,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[apriltag_launch]),
        TimerAction(period=7.0, actions=[ippe_launch]),
        TimerAction(period=9.0, actions=[randomizer]),
        TimerAction(period=10.0, actions=[go_to_pose, navigator, indicator]),
    ])
