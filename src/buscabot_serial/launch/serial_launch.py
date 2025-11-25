from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='buscabot_serial',
            executable='esp32_serial',
            name='esp32_serial_node',
            output='screen'
        )
    ])
