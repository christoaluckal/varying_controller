from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    parameter = os.path.join(get_package_share_directory('varying_controller'), 'config', 'rand_signal_params.yaml')

    signal_node = Node(
        package='varying_controller',
        executable='run_signal_generator',
        name='signal_generator_node',
        output='screen',
        parameters=[parameter]
    )

    # ld.add_action(signal_node)

    ld.add_action(signal_node)

    return ld