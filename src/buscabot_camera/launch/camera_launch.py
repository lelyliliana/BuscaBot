from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='buscabot_camera',
            output='screen',
            parameters=[{
                'image_size': [640, 480],
                'frame_rate': 30,
            }],
            # Si más adelante queremos remappings, van aquí dentro,
            # por ahora lo dejamos simple y correcto.
        ),
    ])

