#!/usr/bin/env python3
"""
KISS-ICP launch 파일 - Local Map 시각화 포함
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # KITTI 플레이어 노드
    player_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_ros_node",
        name="kitti_player",
        arguments=["player"],
        parameters=[
            {
                "dataset_path": "/home/hyeon/kiss_icp_ws/src/data_odometry_velodyne/dataset/sequences/00",
                "publish_rate": 3.0,
                "frame_id": "velodyne",
                "loop_playback": True,
                "auto_start": True,
            }
        ],
        output="screen",
    )

    # KISS-ICP 노드 (namespace와 토픽 매핑 적용)
    kiss_icp_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_ros_node",
        name="kiss_icp",
        namespace="kiss_icp",  # namespace 추가
        parameters=[
            {
                "max_range": 100.0,
            }
        ],
        remappings=[
            # 토픽 매핑 설정
            ("pointcloud", "/pointcloud"),  # 입력: 글로벌 토픽 사용
            ("odometry", "/kiss_icp/odometry"),  # 출력: namespace 포함
            ("path", "/kiss_icp/trajectory"),  # 출력: RViz 설정과 일치
            ("local_map", "/kiss_icp/local_map"),  # 출력: RViz 설정과 일치
        ],
        output="screen",
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_velodyne_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "velodyne"],
    )

    # RViz2 노드 (설정 파일 포함)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", "/path/to/kiss_icp_evaluation.rviz"],  # RViz 설정 파일 경로
        output="screen",
    )

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "dataset_path",
                default_value="/home/hyeon/kiss_icp_ws/src/data_odometry_velodyne/dataset/sequences/00",
                description="Path to KITTI dataset sequence",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="3.0",
                description="Rate to publish point clouds (Hz)",
            ),
            DeclareLaunchArgument(
                "max_range",
                default_value="100.0",
                description="Maximum range for KISS-ICP processing",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 for visualization",
            ),
            # Nodes
            player_node,
            kiss_icp_node,
            static_tf,
            rviz_node,
        ]
    )
