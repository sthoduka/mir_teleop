import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('mir_teleop'),
        'config',
        'teleop_twist_joy_node.yaml'
        )
    print(config)

    node=Node(
        package = 'teleop_twist_joy',
        name = 'teleop_twist',
        executable = 'teleop_node',
        output='screen',
        parameters = [config]
        )

    ld.add_action(node)

    return ld
