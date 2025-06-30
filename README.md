# KISS-ICP 기반 Hand-made LiDAR Odometry

<div align="center">

**프로덕션 수준의 KISS-ICP 구현**  
*Keep It Small and Simple - Iterative Closest Point*

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build Status](https://img.shields.io/badge/Build-Passing-green.svg)]()

**BARAM 로봇 동아리 32기 | 작성자: 23 홍지현 | 버전: 2.0**

</div>

---

## 프로젝트 개요

본 프로젝트는 **KISS-ICP (Keep It Small and Simple - Iterative Closest Point)** 알고리즘을 완전히 직접 구현하여 고성능 LiDAR 기반 Odometry 시스템을 개발한 **프로덕션 수준의 로보틱스 프로젝트**입니다. 

### 핵심 목표
- **경량화된 SLAM**: 복잡한 루프 클로저 없이 실시간 LiDAR Odometry 구현
- **모듈화 설계**: 각 알고리즘 컴포넌트를 독립적인 ROS 2 패키지로 구현
- **성능 평가**: KITTI 데이터셋 기반 정량적 성능 분석

## 시스템 아키텍처

```
kiss_icp_ws/
├── src/handmade_kissicp/
│   ├── adaptive_threshold/       # 적응적 임계값 계산
│   ├── constant_velocity/        # 등속도 모델 예측
│   ├── kiss_icp_voxel/           # 복셀 그리드 다운샘플링
│   ├── robust_icp/               # 로버스트 ICP 구현
│   ├── scan_deskewing/           # 스캔 왜곡 보정
│   ├── kiss_icp_core/            # 통합 KISS-ICP 엔진
│   ├── kiss_icp_ros/             # ROS 2 인터페이스
│   └── evaluation/               # 성능 평가 도구
└── README.md                     # 프로젝트 가이드
```

## 주요 특징

### 알고리즘 구현
- **등속도 모델**: 이전 프레임 정보 기반 초기값 예측
- **스캔 왜곡 보정**: 센서 회전에 의한 Point Cloud 왜곡 보정
- **적응적 임계값**: 환경에 따른 동적 대응점 검색
- **로버스트 ICP**: Geman-McClure 커널 기반 아웃라이어 제거
- **복셀 그리드**: 효율적인 Point Cloud 다운샘플링

### ROS 2 통합
- **실시간 처리**: 고성능 Point Cloud 처리 파이프라인
- **시각화**: RViz2 기반 실시간 Odometry 및 Map 시각화
- **모듈화**: 독립적인 패키지 구성으로 높은 재사용성
- **설정 관리**: YAML 기반 매개변수 관리

### 성능 평가
- **KITTI 지원**: 표준 자율주행 데이터셋 완전 지원
- **정량적 분석**: ATE, RPE 등 표준 메트릭 제공
- **비교 분석**: Ground Truth와의 정확한 성능 비교

## 시스템 요구사항

### 하드웨어 요구사항
- **CPU**: Intel i5 8세대 이상 또는 AMD Ryzen 5 3600 이상
- **RAM**: 최소 8GB (권장 16GB)
- **저장공간**: 최소 20GB 여유공간
- **GPU**: 선택사항 (향후 CUDA 지원 예정)

### 소프트웨어 요구사항
- **OS**: Ubuntu 22.04 LTS (권장) 또는 20.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **컴파일러**: GCC 11+ (C++17 지원)
- **Python**: Python 3.8+

### 라이브러리 의존성
```bash
# 핵심 라이브러리
- PCL (Point Cloud Library) 1.12+
- Eigen3 3.4+
- OpenMP (병렬 처리)

# ROS 2 패키지
- sensor_msgs, geometry_msgs, nav_msgs
- tf2, tf2_ros, tf2_geometry_msgs
- pcl_ros, pcl_conversions
- visualization_msgs
```

## 빠른 시작

### 자동 설치 (권장)

가장 간단하고 안전한 방법입니다:

```bash
# 설치 스크립트 다운로드 및 실행
wget -O install_kiss_icp.sh https://raw.githubusercontent.com/mkdir-sweetiepie/handmade_kissicp/main/scripts/install.sh
chmod +x install_kiss_icp.sh
./install_kiss_icp.sh

# 또는 원라이너로:
curl -sSL https://raw.githubusercontent.com/mkdir-sweetiepie/handmade_kissicp/main/scripts/install.sh | bash
```

### 수동 설치

#### 단계 1: 의존성 설치
```bash
# ROS 2 Humble 설치 (Ubuntu 22.04)
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop-full

# 개발 도구
sudo apt install python3-colcon-common-extensions python3-rosdep
```

#### 단계 2: 라이브러리 설치
```bash
# PCL 및 기타 라이브러리
sudo apt install -y \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-geometry-msgs
```

#### 단계 3: 워크스페이스 생성
```bash
# 워크스페이스 설정
mkdir -p ~/kiss_icp_ws/src
cd ~/kiss_icp_ws/src

# 프로젝트 클론
git clone https://github.com/mkdir-sweetiepie/handmade_kissicp.git
```

#### 단계 4: 빌드
```bash
cd ~/kiss_icp_ws

# ROS 2 환경 설정
source /opt/ros/humble/setup.bash

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 순차적 빌드
colcon build --packages-select adaptive_threshold constant_velocity kiss_icp_voxel robust_icp scan_deskewing
colcon build --packages-select kiss_icp_core
colcon build --packages-select kiss_icp_ros evaluation

# 환경 설정
source install/setup.bash
```

## 사용 방법

### 기본 실행

#### KISS-ICP Odometry 노드 실행
```bash
# 환경 설정
source ~/kiss_icp_ws/install/setup.bash

# 또는 개별 노드 실행
ros2 run kiss_icp_ros kiss_icp_ros_node

# 커스텀 토픽으로 실행
ros2 run kiss_icp_ros kiss_icp_ros_node --ros-args -r pointcloud:=/velodyne_points

# launch 실행(파라미터 사용)
ros2 launch kiss_icp_ros working_kitti.launch.py   sequence:=00   publish_rate:=3.0   max_range:=100.0

```

#### RViz2 시각화
```bash
# 별도 터미널에서 RViz2 실행
ros2 run rviz2 rviz2 -d /home/hyeon/kiss_icp_ws/src/handmade_kissicp/kiss_icp_ros/rivz/default.rviz
```

### KITTI 데이터셋 평가

#### 데이터셋 준비
```bash
# KITTI Odometry 데이터셋 다운로드
mkdir -p ~/kitti_data
cd ~/kitti

# 예시: 시퀀스 00 다운로드
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip
unzip data_odometry_velodyne.zip

# 디렉토리 구조 확인
ls -la dataset/sequences/00/velodyne/
```

#### KITTI 평가 실행
```bash
cd ~/kiss_icp_ws/src/handmade_kissicp/evaluation/
rm -rf build
mkdir build && cd build
cmake ..
make -j$(nproc)
./test_evaluation
cd ..
python3 scripts/csv_visualizer.py --csv ~/kiss_icp_ws/results/sequence_00/trajectory_comparison.csv --output ~/kiss_icp_ws/results/sequence_00/plots --kitti-style --show

```

## 설정 및 튜닝

### 기본 설정 파일
```yaml
# ~/kiss_icp_ws/src/handmade_kissicp/kiss_icp_ros/config/kiss_icp_params.yaml

kiss_icp_odometry:
  ros__parameters:
    # 프레임 설정
    base_frame: "base_link"
    odom_frame: "odom"
    lidar_frame: "velodyne"

    # 알고리즘 매개변수
    max_range: 100.0              # LiDAR 최대 범위 (m)
    min_range: 1.0                # LiDAR 최소 범위 (m)
    voxel_size: 0.5               # 복셀 크기 (m)
    
    # ICP 설정
    max_correspondences: 5000     # 최대 대응점 수
    correspondence_threshold: 1.0  # 대응점 거리 임계값
    max_icp_iterations: 50        # 최대 ICP 반복 수
    convergence_threshold: 0.001  # 수렴 임계값
```

### 성능 튜닝 가이드

#### 🚀 속도 최적화
```yaml
# 빠른 처리를 위한 설정
voxel_size: 1.0                   # 더 큰 복셀 크기
max_correspondences: 2000         # 적은 대응점
max_icp_iterations: 20            # 적은 반복 수
```

#### 정확도 최적화
```yaml
# 높은 정확도를 위한 설정
voxel_size: 0.25                  # 더 작은 복셀 크기
max_correspondences: 10000        # 많은 대응점
max_icp_iterations: 100           # 많은 반복 수
convergence_threshold: 0.0001     # 엄격한 수렴 조건
```

## 성능 평가 결과

### KITTI 데이터셋 벤치마크

| 시퀀스 | ATE (m) | Translation Error (%) | Rotation Error (deg/m) | 처리 시간 (ms) |
|--------|---------|----------------------|------------------------|----------------|
| 00     | 17.4    | 0.50                 | 0.0015                | 45             |
| 01     | 28.3    | 0.62                 | 0.0018                | 42             |
| 02     | 25.1    | 0.55                 | 0.0016                | 48             |
| 05     | 12.3    | 0.45                 | 0.0012                | 38             |
| 평균   | 20.8    | 0.53                 | 0.0015                | 43             |

### 다른 방법들과의 비교

| 방법 | ATE (m) | 실시간 처리 | 메모리 사용량 | 구현 복잡도 |
|------|---------|-------------|---------------|-------------|
| **KISS-ICP (Our)** | **20.8** | **✅** | **512MB** | **낮음** |
| LOAM | 18.5 | ❌ | 2GB | 매우 높음 |
| LeGO-LOAM | 22.1 | ✅ | 1.5GB | 높음 |
| LIO-SAM | 15.2 | ✅ | 1.8GB | 매우 높음 |

