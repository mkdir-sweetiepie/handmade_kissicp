#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for KITTI odometry evaluation."""

    # Package directories
    pkg_share = get_package_share_directory("kiss_icp_ros")
    config_dir = os.path.join(pkg_share, "config")

    # Launch arguments
    dataset_path_arg = DeclareLaunchArgument(
        "dataset_path",
        default_value="/path/to/kitti/dataset",
        description="Path to KITTI dataset root directory",
    )

    sequence_arg = DeclareLaunchArgument(
        "sequence",
        default_value="00",
        description="KITTI sequence number (e.g., 00, 01, ...)",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="Point cloud publishing rate in Hz",
    )

    loop_playback_arg = DeclareLaunchArgument(
        "loop_playback",
        default_value="false",
        description="Whether to loop the playback",
    )

    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Whether to start playback automatically",
    )

    max_range_arg = DeclareLaunchArgument(
        "max_range", default_value="80.0", description="Maximum LiDAR range in meters"
    )

    voxel_size_arg = DeclareLaunchArgument(
        "voxel_size",
        default_value="0.0",  # Will be set to 0.01 * max_range
        description="Voxel size for downsampling (0.0 for auto)",
    )

    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="Whether to publish TF transforms",
    )

    publish_map_arg = DeclareLaunchArgument(
        "publish_map", default_value="true", description="Whether to publish local map"
    )

    debug_mode_arg = DeclareLaunchArgument(
        "debug_mode",
        default_value="false",
        description="Enable debug mode for detailed logging",
    )

    def launch_setup(context, *args, **kwargs):
        """Setup launch nodes with resolved parameters."""

        # KITTI player node
        kitti_player_node = Node(
            package="kiss_icp_ros",
            executable="kitti_player_node",
            name="kitti_player",
            output="screen",
            parameters=[
                {
                    "dataset_path": LaunchConfiguration("dataset_path"),
                    "sequence": LaunchConfiguration("sequence"),
                    "publish_rate": LaunchConfiguration("publish_rate"),
                    "loop_playback": LaunchConfiguration("loop_playback"),
                    "auto_start": LaunchConfiguration("auto_start"),
                    "frame_id": "velodyne",
                    "cloud_topic": "/velodyne_points",
                }
            ],
        )

        # KISS-ICP odometry node
        kiss_icp_node = Node(
            package="kiss_icp_ros",
            executable="kiss_icp_node",
            name="kiss_icp",
            output="screen",
            parameters=[
                os.path.join(config_dir, "kiss_icp_params.yaml"),
                {
                    "max_range": LaunchConfiguration("max_range"),
                    "voxel_size": LaunchConfiguration("voxel_size"),
                    "publish_tf": LaunchConfiguration("publish_tf"),
                    "publish_map": LaunchConfiguration("publish_map"),
                    "cloud_topic": "/velodyne_points",
                    "base_frame": "base_link",
                    "odom_frame": "odom",
                    "map_frame": "map",
                },
            ],
            remappings=[
                ("/velodyne_points", "/velodyne_points"),
                ("/kiss_icp/odometry", "/odometry/kiss_icp"),
                ("/kiss_icp/pose", "/pose/kiss_icp"),
                ("/kiss_icp/path", "/path/kiss_icp"),
                ("/kiss_icp/local_map", "/map/local"),
            ],
        )

        # Optional: Evaluation node for real-time performance monitoring
        evaluation_node = Node(
            package="kiss_icp_ros",
            executable="evaluation_node",
            name="kiss_icp_evaluation",
            output="screen",
            parameters=[
                {
                    "sequence": LaunchConfiguration("sequence"),
                    "dataset_path": LaunchConfiguration("dataset_path"),
                    "evaluation_rate": 1.0,  # Hz
                    "save_results": True,
                    "results_directory": "/tmp/kiss_icp_results",
                }
            ],
            condition=LaunchConfiguration("debug_mode"),  # Only run in debug mode
        )

        return [kitti_player_node, kiss_icp_node, evaluation_node]

    return LaunchDescription(
        [
            dataset_path_arg,
            sequence_arg,
            publish_rate_arg,
            loop_playback_arg,
            auto_start_arg,
            max_range_arg,
            voxel_size_arg,
            publish_tf_arg,
            publish_map_arg,
            debug_mode_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
