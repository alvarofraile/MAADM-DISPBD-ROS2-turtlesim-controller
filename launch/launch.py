from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Nodo controller del paquete turtlesim_controller
        Node(
            package='turtlesim_controller',
            executable='controller',
            name='controller',
        )

    ])

