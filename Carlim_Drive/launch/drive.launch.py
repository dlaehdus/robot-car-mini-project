from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Carlim_key',
            executable='keyboard_node',
            name='keyboard_node',
            output='screen'
        ),
        Node(
            package='Carlim_Drive',
            executable='inwheel_node',
            name='inwheel_node',
            output='screen'
        ),
        Node(
            package='Carlim_Drive',
            executable='steering_node',
            name='steering_node',
            output='screen'
        ),
    ])