"""Launch lmpc_node against either f1tenth_gym_ros or the real car -- the
node itself has no sim/real branches; only pose_topic/drive_topic differ.

Sim (default):
    ros2 launch lmpc_ros2 lmpc.launch.py

Real car (see README.md's real-car section before running this):
    ros2 launch lmpc_ros2 lmpc.launch.py \\
        pose_topic:=/pf/pose/odom map_topic:=/map

Either way, something must already be publishing map_topic (transient-local
OccupancyGrid) before the controller can initialize -- f1tenth_gym_ros needs
a map for its own laser-scan simulation, and any real localization stack
needs one too, so this is normally already satisfied.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lmpc_ros2")
    params_file = os.path.join(pkg_share, "config", "lmpc_params.yaml")
    default_track_dir = os.path.join(pkg_share, "data", "barc_oval")

    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic", default_value="/ego_racecar/odom",
        description="Odometry topic to control from (real car: e.g. /pf/pose/odom)",
    )
    drive_topic_arg = DeclareLaunchArgument(
        "drive_topic", default_value="/drive",
        description="AckermannDriveStamped topic to publish control on",
    )
    map_topic_arg = DeclareLaunchArgument(
        "map_topic", default_value="/map",
        description="OccupancyGrid topic to initialize the controller from",
    )
    track_dir_arg = DeclareLaunchArgument(
        "track_dir", default_value=default_track_dir,
        description="Directory containing <track_name>_waypoints.csv and "
                     "<track_name>_initial_safe_set.csv (default: bundled barc_oval)",
    )
    track_name_arg = DeclareLaunchArgument(
        "track_name", default_value="barc_oval",
        description="File prefix within track_dir -- must match track_dir's "
                     "own track when overriding both (e.g. pure_pursuit_node's "
                     "output_csv uses the same convention, see "
                     "pure_pursuit.launch.py)",
    )

    track_dir = LaunchConfiguration("track_dir")
    track_name = LaunchConfiguration("track_name")

    lmpc_node = Node(
        package="lmpc_ros2",
        executable="lmpc_node",
        name="lmpc_node",
        output="screen",
        parameters=[
            params_file,
            {
                "pose_topic": LaunchConfiguration("pose_topic"),
                "drive_topic": LaunchConfiguration("drive_topic"),
                "map_topic": LaunchConfiguration("map_topic"),
                "waypoint_csv": [track_dir, "/", track_name, "_waypoints.csv"],
                "init_safe_set_csv": [track_dir, "/", track_name, "_initial_safe_set.csv"],
            },
        ],
    )

    return LaunchDescription([
        pose_topic_arg,
        drive_topic_arg,
        map_topic_arg,
        track_dir_arg,
        track_name_arg,
        lmpc_node,
    ])
