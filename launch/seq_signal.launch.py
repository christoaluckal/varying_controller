from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    param_file = LaunchConfiguration('param_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=os.path.join(get_package_share_directory('varying_controller'), 'config', 'seq_signal_params_const.yaml'),
            description='Path to the parameter file'
        ),
        Node(
            package='varying_controller',
            executable='run_seq_generator',
            name='generate_seq_signal_node',
            output='screen',
            parameters=[param_file]
        )
    ])