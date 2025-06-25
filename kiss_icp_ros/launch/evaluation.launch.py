#!/usr/bin/env python3
"""
간소화된 KISS-ICP 평가 시스템 (실제 빌드된 executable만 사용)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 런치 아규먼트들
    dataset_path_arg = DeclareLaunchArgument(
        "dataset_path",
        default_value="/home/hyeon/kiss_icp_ws/src/data_odometry_velodyne/dataset/sequences",
        description="KITTI 데이터셋 경로",
    )

    sequence_arg = DeclareLaunchArgument(
        "sequence", default_value="05", description="평가할 KITTI 시퀀스 번호"
    )

    results_path_arg = DeclareLaunchArgument(
        "results_path",
        default_value="./evaluation_results",
        description="평가 결과 저장 경로",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="RViz 사용 여부"
    )

    # 1. KITTI 플레이어 (실제 존재하는 executable)
    kitti_player_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_ros_node",  # 실제 빌드된 executable
        name="kitti_player",
        arguments=["player"],
        parameters=[
            {
                "data_path": PathJoinSubstitution(
                    [
                        LaunchConfiguration("dataset_path"),
                        LaunchConfiguration("sequence"),
                    ]
                ),
                "publish_rate": 2.0,
                "frame_id": "velodyne",
                "loop": False,
                "max_frames": 1000,  # 평가용
            }
        ],
        output="screen",
    )

    # 2. KISS-ICP (실제 존재하는 executable)
    kiss_icp_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_ros_node",  # 실제 빌드된 executable
        name="kiss_icp_evaluation",
        parameters=[
            {
                # 기본 프레임 설정
                "odom_frame": "odom",
                "base_frame": "base_link",
                # 평가 최적화 파라미터
                "max_range": 50.0,
                "voxel_size": 0.5,
                "max_iterations": 50,
                "convergence_criteria": 1e-4,
                # Robust ICP 파라미터
                "scale_parameter": 0.05,
                "robust_kernel": True,
                # Adaptive Threshold 파라미터
                "adaptive_threshold": True,
                "min_motion_threshold": 0.1,
                "sigma_multiplier": 3.0,
                # 평가 전용 옵션
                "verbose": True,
                "debug_info": True,
                "save_trajectory": True,
                "results_path": LaunchConfiguration("results_path"),
            }
        ],
        output="screen",
    )

    # 3. RViz (설정 파일 없이 기본 실행)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # 4. Static TFs
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    static_tf_base_velodyne = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_velodyne",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "velodyne"],
    )

    # 시작 메시지
    start_message = LogInfo(
        msg=[
            "======================================\\n",
            "KISS-ICP 간소화 평가 시스템 시작\\n",
            "======================================\\n",
            "Dataset: ",
            LaunchConfiguration("dataset_path"),
            "\\n",
            "Sequence: ",
            LaunchConfiguration("sequence"),
            "\\n",
            "Results: ",
            LaunchConfiguration("results_path"),
            "\\n",
            "======================================\\n",
        ]
    )

    return LaunchDescription(
        [
            # Arguments
            dataset_path_arg,
            sequence_arg,
            results_path_arg,
            use_rviz_arg,
            # 시작 메시지
            start_message,
            # Static TFs
            static_tf_map_odom,
            static_tf_base_velodyne,
            # 메인 노드들 (실제 존재하는 것만)
            kitti_player_node,
            kiss_icp_node,
            rviz_node,
        ]
    )
