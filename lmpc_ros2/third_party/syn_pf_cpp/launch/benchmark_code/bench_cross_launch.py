"""Cross-process benchmark: synpf_cpp and latency_probe in SEPARATE processes.
Hands off via DDS (Fast DDS shared-memory transport on same host, or UDP
across hosts). Comparison vs intra-process tells you the cost of DDS.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('syn_pf_cpp'),
        'config', 'synpf_cpp_params.yaml')

    pf_node = Node(
        package='syn_pf_cpp',
        executable='synpf_node',
        name='synpf_cpp',
        output='screen',
        parameters=[cfg, {'pose_pub_topic': '/tracked_pose_cpp'}],
        prefix='taskset -c 0',
    )
    probe_node = Node(
        package='syn_pf_cpp',
        executable='latency_probe',
        name='latency_probe_cross',
        output='screen',
        parameters=[{'pose_topic': '/tracked_pose_cpp', 'label': 'CROSS'}],
        prefix='taskset -c 1',
    )
    return LaunchDescription([pf_node, probe_node])
