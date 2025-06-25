# kiss_icp_ros/launch/kiss_icp.launch.py
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
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
            [FindPackageShare("kiss_icp_ros"), "rviz", "kiss_icp.rviz"]
        ),
        description="Path to the RViz configuration file",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Whether to launch RViz"
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/pointcloud",
        description="Input point cloud topic",
    )

    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link", description="Base frame ID"
    )

    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame", default_value="odom", description="Odometry frame ID"
    )

    # KISS-ICP odometry node
    kiss_icp_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_ros_node",
        name="kiss_icp_odometry",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
            },
        ],
        remappings=[
            ("pointcloud", LaunchConfiguration("pointcloud_topic")),
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
            config_file_arg,
            rviz_config_arg,
            use_rviz_arg,
            pointcloud_topic_arg,
            base_frame_arg,
            odom_frame_arg,
            LogInfo(msg="Launching KISS-ICP odometry system"),
            kiss_icp_node,
            rviz_node,
        ]
    )
