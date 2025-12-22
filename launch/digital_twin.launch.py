import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to our package and config
    dt_pkg_dir = get_package_share_directory('digital_twin_pkg')
    objects_config_path = os.path.join(dt_pkg_dir, 'config', 'objects.json')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Launch Carla ROS Bridge
    # We use the standard bridge launch
    carla_ros_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('carla_ros_bridge'), 'carla_ros_bridge.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'timeout': '200', # Higher timeout for heavy load
            'town': 'Town01', # Simpler map for better FPS and visibility
            # 'register_all_sensors': 'false' # We explicitly spawn them
        }.items()
    )

    # 2. Spawn Objects (Vehicle + Sensors)
    carla_spawn_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('carla_spawn_objects'), 'carla_spawn_objects.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': objects_config_path,
            'spawn_sensors_only': 'False'
        }.items()
    )

    # Foxglove Bridge Node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Carla) clock if true'),
        
        carla_ros_bridge_launch,
        carla_spawn_objects_launch,
        foxglove_bridge
    ])
