import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'varying_controller'

    # Build the full path to your actual parameter file
    params_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'joint_signal_controller_params.yaml'
    )

    interactive_signal_node = Node(
        package=package_name,
        executable='run_full_excavator_test', # This now correctly matches setup.py
        name='interactive_signal_generator_node',
        output='screen',
        # Pass the correct file path to the node as a parameter
        parameters=[
            {'params_file': params_file_path}
        ]
    )

    return LaunchDescription([
        interactive_signal_node
    ])
