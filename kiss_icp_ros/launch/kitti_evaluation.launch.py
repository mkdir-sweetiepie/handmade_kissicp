# kiss_icp_ros/launch/kitti_evaluation.launch.py
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    dataset_path_arg = DeclareLaunchArgument(
        "dataset_path",
        default_value="/path/to/kitti/dataset",
        description="Path to KITTI dataset root directory",
    )

    sequence_arg = DeclareLaunchArgument(
        "sequence",
        default_value="00",
        description="KITTI sequence number (e.g., 00, 01, 02, ...)",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="Point cloud publishing rate in Hz",
    )

    loop_playback_arg = DeclareLaunchArgument(
        "loop_playback",
        default_value="true",
        description="Whether to loop the playback",
    )

    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Whether to automatically start playback",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kiss_icp_ros"), "config", "kiss_icp_params.yaml"]
        ),
        description="Path to the KISS-ICP configuration file",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kiss_icp_ros"), "rviz", "kiss_icp_evaluation.rviz"]
        ),
        description="Path to the RViz configuration file",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Whether to launch RViz"
    )

    # KITTI player node
    kitti_player_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_ros_node",
        name="kitti_player",
        arguments=["player"],
        parameters=[
            {
                "dataset_path": LaunchConfiguration("dataset_path"),
                "sequence": LaunchConfiguration("sequence"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "loop_playback": LaunchConfiguration("loop_playback"),
                "auto_start": LaunchConfiguration("auto_start"),
                "frame_id": "velodyne",
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    # KISS-ICP odometry node
    kiss_icp_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_ros_node",
        name="kiss_icp_odometry",
        parameters=[LaunchConfiguration("config_file")],
        remappings=[
            ("pointcloud", "/pointcloud"),
        ],
        output="screen",
        emulate_tty=True,
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            dataset_path_arg,
            sequence_arg,
            publish_rate_arg,
            loop_playback_arg,
            auto_start_arg,
            config_file_arg,
            rviz_config_arg,
            use_rviz_arg,
            LogInfo(msg="Launching KITTI evaluation system"),
            kitti_player_node,
            kiss_icp_node,
            rviz_node,
        ]
    )
