"""Drive a centerline via pure pursuit and record a fresh initial-safe-set
CSV for lmpc_node -- run this FIRST for any new track, then lmpc.launch.py
against the same track_dir/track_name. No sim/real branching, same as
lmpc.launch.py -- only pose_topic/drive_topic differ.

Sim, default (bundled gold_conference_room) track:
    ros2 launch lmpc_ros2 pure_pursuit.launch.py

New track (needs <track_dir>/<track_name>_centerline.csv already present --
see ../README.md Section 3):
    ros2 launch lmpc_ros2 pure_pursuit.launch.py \\
        track_dir:=/path/to/your/track_dir track_name:=my_track

Real car. pose_source defaults to "odom" (trust pose_topic's Odometry
pose+v directly, the sim behavior) -- set it to "pf" to instead take x/y/yaw
from a PoseStamped particle-filter topic (pf_pose_topic), with pose_topic
then only used for its |v|. Same split lmpc.launch.py uses:
    ros2 launch lmpc_ros2 pure_pursuit.launch.py \\
        pose_source:=pf pose_topic:=/odom pf_pose_topic:=/tracked_pose \\
        track_dir:=/path/to/venue track_name:=venue

max_speed defaults from config/lmpc_params.yaml (self-contained under this
package) -- pass max_speed:=<value> explicitly to override it for a one-off
run.

The node exits on its own once recording finishes (default 2 laps) -- that's
the signal to stop and launch lmpc_node next.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("lmpc_ros2")
    params_file = os.path.join(pkg_share, "config", "lmpc_params.yaml")

    track_name = LaunchConfiguration("track_name").perform(context)
    track_dir = LaunchConfiguration("track_dir").perform(context)
    if not track_dir:
        # NOT a static default -- resolved after track_name is known, same
        # reasoning as lmpc.launch.py's track_dir.
        track_dir = os.path.join(pkg_share, "data", track_name)

    overrides = {
        "pose_topic": LaunchConfiguration("pose_topic"),
        "pose_source": LaunchConfiguration("pose_source"),
        "pf_pose_topic": LaunchConfiguration("pf_pose_topic"),
        "drive_topic": LaunchConfiguration("drive_topic"),
        "centerline_csv": os.path.join(track_dir, f"{track_name}_centerline.csv"),
        "output_csv": os.path.join(track_dir, f"{track_name}_initial_safe_set.csv"),
        "laps": LaunchConfiguration("laps"),
    }
    max_speed_override = LaunchConfiguration("max_speed").perform(context)
    if max_speed_override:
        overrides["max_speed"] = float(max_speed_override)

    pure_pursuit_node = Node(
        package="lmpc_ros2",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        output="screen",
        parameters=[params_file, overrides],
    )
    return [pure_pursuit_node]


def generate_launch_description():
    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic", default_value="/ego_racecar/odom",
        description="Odometry topic. pose_source=odom (default): supplies "
                     "pose AND v directly. pose_source=pf: only its |v| is "
                     "used -- x/y/yaw come from pf_pose_topic instead.",
    )
    pose_source_arg = DeclareLaunchArgument(
        "pose_source", default_value="odom",
        description="'odom' (default, sim-compatible): trust pose_topic's "
                     "Odometry pose+v directly. 'pf' (real car): x/y/yaw "
                     "from pf_pose_topic (PoseStamped) instead.",
    )
    pf_pose_topic_arg = DeclareLaunchArgument(
        "pf_pose_topic", default_value="/tracked_pose",
        description="PoseStamped particle-filter output (e.g. syn_pf_cpp's "
                     "/tracked_pose). Only read when pose_source:=pf.",
    )
    drive_topic_arg = DeclareLaunchArgument(
        "drive_topic", default_value="/drive",
        description="AckermannDriveStamped topic to publish control on",
    )
    track_dir_arg = DeclareLaunchArgument(
        "track_dir", default_value="",
        description="Directory containing <track_name>_centerline.csv; "
                     "<track_name>_initial_safe_set.csv is written here too. "
                     "Empty (default) resolves to this package's own "
                     "data/<track_name>/ share directory.",
    )
    track_name_arg = DeclareLaunchArgument(
        "track_name", default_value="gold_conference_room",
        description="File prefix within track_dir (same convention as lmpc.launch.py)",
    )
    max_speed_arg = DeclareLaunchArgument(
        "max_speed", default_value="",
        description="Speed cap [m/s] override. Empty (default) uses "
                     "config/lmpc_params.yaml's max_speed instead.",
    )
    laps_arg = DeclareLaunchArgument(
        "laps", default_value="2",
        description="Laps to record before stopping (matches LMPCCore's own "
                     "2-iteration startup requirement)",
    )

    return LaunchDescription([
        pose_topic_arg,
        pose_source_arg,
        pf_pose_topic_arg,
        drive_topic_arg,
        track_dir_arg,
        track_name_arg,
        max_speed_arg,
        laps_arg,
        OpaqueFunction(function=launch_setup),
    ])
