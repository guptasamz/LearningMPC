import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('syn_pf_cpp'),
        'config', 'synpf_cpp_real_params.yaml')
    return LaunchDescription([
        Node(
            package='syn_pf_cpp',
            executable='synpf_node',
            name='synpf_cpp',
            output='screen',
            parameters=[cfg],
            prefix='taskset -c 0',
        ),
    ])
