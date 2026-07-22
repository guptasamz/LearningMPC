"""Intra-process benchmark: synpf_cpp + latency_probe in a single
ComposableNodeContainer with intra-process comms enabled. Zero-copy
shared-memory hand-off between pub/sub.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('syn_pf_cpp'),
        'config', 'synpf_cpp_params.yaml')

    container = ComposableNodeContainer(
        name='pf_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='syn_pf_cpp',
                plugin='syn_pf_cpp::SynPFComponent',
                name='synpf_cpp',
                parameters=[cfg, {'pose_pub_topic': '/tracked_pose_cpp'}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='syn_pf_cpp',
                plugin='syn_pf_cpp::LatencyProbe',
                name='latency_probe_intra',
                parameters=[{'pose_topic': '/tracked_pose_cpp',
                             'label': 'INTRA'}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        prefix='taskset -c 0,1',
    )
    return LaunchDescription([container])
