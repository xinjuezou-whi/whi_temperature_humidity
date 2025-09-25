from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('whi_temperature_humidity'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='whi_temperature_humidity',
            executable='whi_temperature_humidity_node',
            name='whi_temperature_humidity',
            output='screen',
            parameters=[config]
        )
    ])
