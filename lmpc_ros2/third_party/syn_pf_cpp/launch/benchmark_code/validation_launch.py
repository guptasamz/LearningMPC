"""Run Python SynPF and C++ SynPF in parallel + comparison node.

Python SynPF publishes to /tracked_pose_py; C++ SynPF publishes to /tracked_pose_cpp.
Both consume same /map + /scan + /ego_racecar/odom (sim) inputs.
pf_compare logs aligned poses + rolling diff stats.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    py_pkg_share  = get_package_share_directory('syn_particle_filter')
    cpp_pkg_share = get_package_share_directory('syn_pf_cpp')

    py_params  = os.path.join(py_pkg_share,  'config', 'pf2_params_sim.yaml')
    cpp_params = os.path.join(cpp_pkg_share, 'config', 'synpf_cpp_params.yaml')

    py_node = Node(
        package='syn_particle_filter',
        executable='synpf_node',
        name='synpf',  # MUST match yaml key 'synpf:'
        output='screen',
        parameters=[
            py_params,
            {'pose_pub_topic': '/tracked_pose_py',
             'publish_tf': False,
             'viz': False},
        ],
        prefix='taskset -c 0',
    )
    cpp_node = Node(
        package='syn_pf_cpp',
        executable='synpf_node',
        name='synpf_cpp',
        output='screen',
        parameters=[
            cpp_params,
            {'pose_pub_topic': '/tracked_pose_cpp'},
        ],
        prefix='taskset -c 1',
    )
    compare_node = Node(
        package='syn_pf_cpp',
        executable='compare_pf_node.py',
        name='pf_compare',
        output='screen',
        parameters=[{
            'py_topic':  '/tracked_pose_py',
            'cpp_topic': '/tracked_pose_cpp',
            'sync_tolerance_ms': 30.0,
            'report_period_s': 10.0,
        }],
    )

    return LaunchDescription([py_node, cpp_node, compare_node])
