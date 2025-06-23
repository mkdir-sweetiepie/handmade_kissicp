#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 런치 아규먼트 선언
    dataset_path_arg = DeclareLaunchArgument(
        "dataset_path",
        default_value="/path/to/kitti/dataset",
        description="KITTI 데이터셋 경로",
    )

    results_path_arg = DeclareLaunchArgument(
        "results_path",
        default_value="./evaluation_results",
        description="평가 결과 저장 경로",
    )

    sequence_arg = DeclareLaunchArgument(
        "sequence", default_value="00", description="평가할 KITTI 시퀀스 번호"
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kiss_icp_ros"), "config", "kiss_icp_params.yaml"]
        ),
        description="KISS-ICP 설정 파일 경로",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kiss_icp_ros"), "rviz", "kiss_icp_evaluation.rviz"]
        ),
        description="RViz 설정 파일 경로",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="RViz 사용 여부"
    )

    auto_start_arg = DeclareLaunchArgument(
        "auto_start", default_value="false", description="자동으로 평가 시작"
    )

    playback_speed_arg = DeclareLaunchArgument(
        "playback_speed", default_value="1.0", description="데이터 재생 속도"
    )

    save_results_arg = DeclareLaunchArgument(
        "save_results", default_value="true", description="결과 저장 여부"
    )

    # KITTI 플레이어 노드
    kitti_player_node = Node(
        package="kiss_icp_ros",
        executable="kitti_player_node",
        name="kitti_player",
        parameters=[
            {
                "dataset_path": LaunchConfiguration("dataset_path"),
                "sequence": LaunchConfiguration("sequence"),
                "playback_speed": LaunchConfiguration("playback_speed"),
                "auto_start": LaunchConfiguration("auto_start"),
                "loop_playback": False,
                "publish_tf": True,
                "publish_ground_truth": True,
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    # KISS-ICP 노드
    kiss_icp_node = Node(
        package="kiss_icp_ros",
        executable="kiss_icp_node",
        name="kiss_icp_odometry",
        parameters=[LaunchConfiguration("config_file")],
        remappings=[
            ("pointcloud", "/kitti/pointcloud"),
            ("odometry", "/kiss_icp/odometry"),
            ("local_map", "/kiss_icp/local_map"),
            ("current_scan", "/kiss_icp/current_scan"),
        ],
        output="screen",
        emulate_tty=True,
    )

    # 평가 노드
    evaluation_node = Node(
        package="kiss_icp_ros",
        executable="evaluation_node",
        name="odometry_evaluator",
        parameters=[
            {
                "results_path": LaunchConfiguration("results_path"),
                "sequence": LaunchConfiguration("sequence"),
                "save_results": LaunchConfiguration("save_results"),
                "evaluation_frequency": 1.0,  # Hz
                "min_poses_for_evaluation": 10,
            }
        ],
        remappings=[
            ("estimated_odometry", "/kiss_icp/odometry"),
            ("ground_truth_odometry", "/kitti/ground_truth"),
        ],
        output="screen",
        emulate_tty=True,
    )

    # RViz 노드
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # Static TF: map -> odom
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # Static TF: base_link -> velodyne
    static_tf_base_velodyne = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_velodyne",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "velodyne"],
    )

    # 평가 완료 후 결과 시각화 (지연 실행)
    visualization_process = TimerAction(
        period=10.0,  # 10초 후 실행
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("kiss_icp_ros"),
                            "scripts",
                            "plot_evaluation_results.py",
                        ]
                    ),
                    "--results_path",
                    LaunchConfiguration("results_path"),
                    "--sequence",
                    LaunchConfiguration("sequence"),
                ],
                output="screen",
            )
        ],
    )

    # 시작 메시지
    start_message = LogInfo(
        msg=[
            "======================================\\n",
            "KISS-ICP 평가 시스템 시작\\n",
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
            # 런치 아규먼트
            dataset_path_arg,
            results_path_arg,
            sequence_arg,
            config_file_arg,
            rviz_config_arg,
            use_rviz_arg,
            auto_start_arg,
            playback_speed_arg,
            save_results_arg,
            # 시작 메시지
            start_message,
            # Static TF
            static_tf_map_odom,
            static_tf_base_velodyne,
            # 메인 노드들
            kitti_player_node,
            kiss_icp_node,
            evaluation_node,
            # RViz (옵션)
            rviz_node,
            # 결과 시각화 (지연 실행)
            # visualization_process,  # 필요시 주석 해제
        ]
    )
