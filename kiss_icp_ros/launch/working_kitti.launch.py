#!/usr/bin/env python3
"""
수정된 KISS-ICP launch 파일 - ICP 발산 문제 해결
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # KITTI 플레이어 노드 (안정적인 속도)
    player_node = Node(
        package='kiss_icp_ros',
        executable='kiss_icp_ros_node',
        name='kitti_player_optimized',
        arguments=['player'],
        parameters=[{
            'data_path': '/home/hyeon/kiss_icp_ws/src/data_odometry_velodyne/dataset/sequences/05',
            'publish_rate': 1.0,    # 2Hz → 1Hz로 더 안정하게
            'frame_id': 'velodyne',
            'loop': False,
            'start_frame': 0,       # 첫 번째 프레임부터 시작
            'max_frames': 100       # 테스트용으로 100프레임만
        }],
        output='screen',
    )

    # KISS-ICP 노드 (ICP 발산 방지 파라미터)
    kiss_icp_node = Node(
        package='kiss_icp_ros',
        executable='kiss_icp_ros_node',
        name='kiss_icp_optimized',
        parameters=[{
            # 기본 프레임 설정
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            
            # 핵심 ICP 파라미터 (발산 방지)
            'max_range': 30.0,           # 60 → 30으로 줄임 (안정성 향상)
            'voxel_size': 0.3,           # 0.5 → 0.3으로 세밀화
            'max_iterations': 15,        # ICP 최대 반복 횟수 제한
            'convergence_criteria': 5e-4, # 수렴 조건 완화
            
            # Robust ICP 파라미터
            'scale_parameter': 0.05,     # Geman-McClure 커널 스케일
            'robust_kernel': True,       # 강인 커널 활성화
            
            # Adaptive Threshold 파라미터
            'adaptive_threshold': True,   # 적응적 임계값 사용
            'min_motion_threshold': 0.1, # 최소 움직임 임계값
            'sigma_multiplier': 2.5,     # 3σ → 2.5σ로 완화
            
            # 다운샘플링 파라미터
            'downsample_ratio': 0.8,     # 다운샘플링 비율
            'min_points_threshold': 1000, # 최소 포인트 수
            
            # 초기화 및 안정성 파라미터
            'scan_duration': 0.1,
            'min_deviation': 0.3,        # 0.5 → 0.3으로 더 민감하게
            'initial_threshold': 2.0,    # 초기 대응점 검색 거리
            'max_correspondence_distance': 1.0, # 최대 대응점 거리
            
            # 디버그 옵션
            'verbose': True,             # 상세 로그 출력
            'debug_info': True,          # 디버그 정보 출력
        }],
        output='screen',
    )

    # Static TF (변경 없음)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velodyne_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
    )

    # 포인트클라우드 전처리 노드 (추가)
    preprocess_node = Node(
        package='kiss_icp_ros',
        executable='pointcloud_preprocessor',
        name='pointcloud_preprocessor',
        parameters=[{
            'filter_outliers': True,     # 이상치 제거
            'outlier_radius': 0.5,       # 이상치 검출 반경
            'outlier_min_neighbors': 3,  # 최소 이웃 수
            'range_filter': True,        # 거리 필터
            'min_range': 1.0,           # 최소 거리
            'max_range': 30.0,          # 최대 거리
        }],
        remappings=[
            ('input', '/pointcloud_raw'),
            ('output', '/pointcloud_filtered'),
        ],
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug mode'
        ),
        DeclareLaunchArgument(
            'slow_mode',
            default_value='true',
            description='Use slow processing mode for stability'
        ),
        
        # Nodes
        player_node,
        kiss_icp_node,
        static_tf,
        # preprocess_node,  # 필요시 주석 해제
    ])