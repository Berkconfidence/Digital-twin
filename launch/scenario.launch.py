from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Yaya oluşturma düğümü
        Node(
            package='digital_twin_pkg',
            executable='spawn_pedestrians',
            name='spawn_pedestrians_node',
            output='screen',
            parameters=[
                {'number_of_walkers': 400},
                {'host': '127.0.0.1'},
                {'port': 2000}
            ]
        ),
        
        # Çukur oluşturma düğümü
        Node(
            package='digital_twin_pkg',
            executable='spawn_potholes_node',
            name='spawn_potholes_node',
            output='screen'
        ),
        
        # Gelişmiş algılama düğümü (Hız kontrolü ve frenleme dahil)
        Node(
            package='digital_twin_pkg',
            executable='advanced_perception',
            name='advanced_perception_node',
            output='screen'
        )
    ])
