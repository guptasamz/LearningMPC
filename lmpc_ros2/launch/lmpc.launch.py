"""Launch lmpc_node against either f1tenth_gym_ros or the real car -- the
node itself has no sim/real branches; only pose_topic/drive_topic differ.

Sim (default, bundled barc_oval):
    ros2 launch lmpc_ros2 lmpc.launch.py

Other track (needs <track_dir>/<track_name>_waypoints.csv and
_initial_safe_set.csv -- see pure_pursuit.launch.py to generate the latter):
    ros2 launch lmpc_ros2 lmpc.launch.py track_dir:=/path/to/dir track_name:=my_track

Real car (see README.md's real-car section before running this):
    ros2 launch lmpc_ros2 lmpc.launch.py \\
        pose_topic:=/pf/pose/odom map_topic:=/map

Either way, something must already be publishing map_topic (transient-local
OccupancyGrid) before the controller can initialize -- f1tenth_gym_ros needs
a map for its own laser-scan simulation, and any real localization stack
needs one too, so this is normally already satisfied.

Controller tuning (r_accel, osqp_max_iter, ...) lives in
config/lmpc_params.yaml, self-contained under this package -- see that
file's own header comment for how it relates to ../../Lmpc_params.yaml.
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
        # NOT a static default -- must be resolved after track_name is known,
        # or overriding track_name alone would silently keep looking in
        # barc_oval's directory for a different track's files.
        track_dir = os.path.join(pkg_share, "data", track_name)

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
                "waypoint_csv": os.path.join(track_dir, f"{track_name}_waypoints.csv"),
                "init_safe_set_csv": os.path.join(
                    track_dir, f"{track_name}_initial_safe_set.csv"
                ),
            },
        ],
    )
    return [lmpc_node]


def generate_launch_description():
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
        "track_dir", default_value="",
        description="Directory containing <track_name>_waypoints.csv and "
                     "<track_name>_initial_safe_set.csv. Empty (default) "
                     "resolves to this package's own data/<track_name>/ "
                     "share directory.",
    )
    track_name_arg = DeclareLaunchArgument(
        "track_name", default_value="barc_oval",
        description="File prefix within track_dir -- must match track_dir's "
                     "own track when overriding both (e.g. pure_pursuit_node's "
                     "output_csv uses the same convention, see "
                     "pure_pursuit.launch.py)",
    )

    return LaunchDescription([
        pose_topic_arg,
        drive_topic_arg,
        map_topic_arg,
        track_dir_arg,
        track_name_arg,
        OpaqueFunction(function=launch_setup),
    ])
