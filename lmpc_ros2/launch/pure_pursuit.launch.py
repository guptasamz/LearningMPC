"""Drive a centerline via pure pursuit and record a fresh initial-safe-set
CSV for lmpc_node -- run this FIRST for any new track, then lmpc.launch.py
against the same track_dir/track_name. No sim/real branching, same as
lmpc.launch.py -- only pose_topic/drive_topic differ.

Sim, default (bundled barc_oval) track, capped at 2 m/s:
    ros2 launch lmpc_ros2 pure_pursuit.launch.py max_speed:=2.0

New track (needs <track_dir>/<track_name>_centerline.csv already present --
see ../README.md's raceline/centerline generation notes):
    ros2 launch lmpc_ros2 pure_pursuit.launch.py \\
        track_dir:=/path/to/your/track_dir track_name:=my_track max_speed:=2.0

Real car:
    ros2 launch lmpc_ros2 pure_pursuit.launch.py \\
        pose_topic:=/pf/pose/odom max_speed:=1.5 \\
        track_dir:=/path/to/venue track_name:=venue

The node exits on its own once recording finishes (default 2 laps) -- that's
the signal to stop and launch lmpc_node next.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lmpc_ros2")
    default_track_dir = os.path.join(pkg_share, "data", "barc_oval")

    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic", default_value="/ego_racecar/odom",
        description="Odometry topic to drive from (real car: e.g. /pf/pose/odom)",
    )
    drive_topic_arg = DeclareLaunchArgument(
        "drive_topic", default_value="/drive",
        description="AckermannDriveStamped topic to publish control on",
    )
    track_dir_arg = DeclareLaunchArgument(
        "track_dir", default_value=default_track_dir,
        description="Directory containing <track_name>_centerline.csv; "
                     "<track_name>_initial_safe_set.csv is written here too "
                     "-- must match the track_dir lmpc_node is later pointed at",
    )
    track_name_arg = DeclareLaunchArgument(
        "track_name", default_value="barc_oval",
        description="File prefix within track_dir (same convention as lmpc.launch.py)",
    )
    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        description="REQUIRED: hard speed cap [m/s] -- no default, must be set "
                     "explicitly. Keep this low, especially on the real car.",
    )
    laps_arg = DeclareLaunchArgument(
        "laps", default_value="2",
        description="Laps to record before stopping (matches LMPCCore's own "
                     "2-iteration startup requirement)",
    )

    track_dir = LaunchConfiguration("track_dir")
    track_name = LaunchConfiguration("track_name")

    pure_pursuit_node = Node(
        package="lmpc_ros2",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        output="screen",
        parameters=[{
            "pose_topic": LaunchConfiguration("pose_topic"),
            "drive_topic": LaunchConfiguration("drive_topic"),
            "centerline_csv": [track_dir, "/", track_name, "_centerline.csv"],
            "output_csv": [track_dir, "/", track_name, "_initial_safe_set.csv"],
            "max_speed": LaunchConfiguration("max_speed"),
            "laps": LaunchConfiguration("laps"),
        }],
    )

    return LaunchDescription([
        pose_topic_arg,
        drive_topic_arg,
        track_dir_arg,
        track_name_arg,
        max_speed_arg,
        laps_arg,
        pure_pursuit_node,
    ])
