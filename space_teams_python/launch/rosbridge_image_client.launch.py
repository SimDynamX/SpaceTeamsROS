from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import os

def generate_launch_description():
    """Generate launch description for running rosbridge_server and image_client."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge websocket server'
    )
    
    # Include rosbridge websocket launch
    rosbridge_launch = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen', # Optional: directs node output to the screen
            parameters=[
                {'port': 9090}, # Optional: specify the port, default is 9090
                # Add other rosbridge_server parameters here if needed
            ]
        )
    
    # Launch the image client node
    image_client_node = Node(
        package='space_teams_python',
        executable='image_client',
        name='image_client',
        output='screen'
    )

    # example of adding a node to the launch description
    # another_node = Node(
    #     package='your_package (probably space_teams_python)',
    #     executable='your_executable',
    #     name='your_node_name',
    #     output='screen'
    # )

    return LaunchDescription([
        port_arg,
        rosbridge_launch,
        image_client_node # , (make sure to separate with comma if adding a node)
        # another_node
    ])