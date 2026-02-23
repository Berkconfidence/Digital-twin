import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dt_pkg_dir = get_package_share_directory('digital_twin_pkg')
    objects_config_path = os.path.join(dt_pkg_dir, 'config', 'objects.json')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 0. Setup Environment for CARLA Python API
    # This is required for carla_waypoint_publisher to find 'agents' module
    carla_env_setup = AppendEnvironmentVariable(
        'PYTHONPATH',
        '/home/berk/Desktop/carla_packed_linux/PythonAPI/carla',
        separator=':'
    )

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
            # 'register_all_sensors': 'false', # We explicitly spawn them
            'synchronous_mode': 'true',
            'fixed_delta_seconds': '0.1'
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

    # 3. Waypoint Publisher (Visualization)
    carla_waypoint_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
        ),
        launch_arguments={
            'role_name': 'ego_vehicle',
            'publish_all_lane_boundaries': 'True'
        }.items()
    )

    # 4. Manual Control (Kapatıldı - Performans ve Foxglove optimizasyonu için)
    # carla_manual_control_node = Node(
    #     package='carla_manual_control',
    #     executable='carla_manual_control',
    #     name='carla_manual_control_ego_vehicle',
    #     output='screen',
    #     parameters=[
    #         {'role_name': 'ego_vehicle'}
    #     ]
    # )

    # 5. Local Planner (Autonomous Control)
    # We launch only the local_planner to allow the user to manually publish 
    # /carla/ego_vehicle/speed_command without interference from the ad_agent 
    # (which defaults to 0 speed if no target is set).
    carla_local_planner_node = Node(
        package='carla_ad_agent',
        executable='local_planner',
        name='local_planner',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'role_name': 'ego_vehicle',
                'Kp_lateral': 0.9,
                'Ki_lateral': 0.0,
                'Kd_lateral': 0.0,
                'Kp_longitudinal': 0.206,
                'Ki_longitudinal': 0.0206,
                'Kd_longitudinal': 0.515,
                'control_time_step': 0.05
            }
        ]
    )


    # 6. Robot Description (URDF)
    urdf_file_name = 'car.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('digital_twin_pkg'),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
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
        carla_env_setup,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Carla) clock if true'),
        
        carla_ros_bridge_launch,
        carla_spawn_objects_launch,
        carla_waypoint_publisher_launch,
        # carla_manual_control_node,
        carla_local_planner_node,
        robot_state_publisher_node,
        foxglove_bridge
    ])
