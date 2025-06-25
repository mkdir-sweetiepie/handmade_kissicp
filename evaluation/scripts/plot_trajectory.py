#!/usr/bin/env python3
"""
KISS-ICP 궤적 시각화 스크립트 (수정된 버전)
matplotlib 스타일 오류 해결
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
import sys
from typing import List, Tuple, Optional
import glob

class TrajectoryVisualizer:
    def __init__(self, figsize=(15, 10)):
        self.figsize = figsize
        # 스타일 설정 - 안전한 방법으로 수정
        try:
            plt.style.use('seaborn-v0_8-whitegrid')
        except:
            try:
                plt.style.use('seaborn-whitegrid')
            except:
                try:
                    plt.style.use('seaborn')
                except:
                    plt.style.use('default')
                    print("기본 matplotlib 스타일을 사용합니다.")

    def load_trajectory_tum(self, filepath: str) -> np.ndarray:
        """TUM 형식의 궤적 파일 로드 (timestamp x y z qx qy qz qw)"""
        try:
            data = np.loadtxt(filepath)
            if data.shape[1] >= 4:
                return data[:, 1:4]  # x, y, z만 추출
            else:
                raise ValueError(f"TUM 파일 형식이 올바르지 않습니다: {filepath}")
        except Exception as e:
            print(f"TUM 파일 로드 오류 {filepath}: {e}")
            return np.array([])

    def load_trajectory_kitti(self, filepath: str) -> np.ndarray:
        """KITTI 형식의 궤적 파일 로드 (12개 값으로 구성된 변환 행렬)"""
        try:
            poses = []
            with open(filepath, "r") as f:
                for line in f:
                    values = list(map(float, line.strip().split()))
                    if len(values) == 12:
                        # KITTI 형식: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
                        x, y, z = values[3], values[7], values[11]
                        poses.append([x, y, z])
            return np.array(poses)
        except Exception as e:
            print(f"KITTI 파일 로드 오류 {filepath}: {e}")
            return np.array([])

    def load_trajectory(self, filepath: str, format_type: str = "auto") -> np.ndarray:
        """궤적 파일 로드 (형식 자동 감지)"""
        if not os.path.exists(filepath):
            print(f"파일이 존재하지 않습니다: {filepath}")
            return np.array([])

        if format_type == "auto":
            # 파일 내용을 보고 형식 판단
            with open(filepath, "r") as f:
                first_line = f.readline().strip().split()

            if len(first_line) >= 8:  # TUM 형식 (8개 이상 컬럼)
                format_type = "tum"
            elif len(first_line) == 12:  # KITTI 형식 (12개 컬럼)
                format_type = "kitti"
            else:
                print(f"알 수 없는 파일 형식: {filepath}")
                return np.array([])

        if format_type.lower() == "tum":
            return self.load_trajectory_tum(filepath)
        elif format_type.lower() == "kitti":
            return self.load_trajectory_kitti(filepath)
        else:
            print(f"지원하지 않는 형식: {format_type}")
            return np.array([])

    def plot_2d_trajectory(
        self,
        ground_truth: np.ndarray,
        estimated: np.ndarray,
        title: str = "Trajectory Comparison",
        save_path: Optional[str] = None,
    ):
        """2D 궤적 비교 플롯"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=self.figsize)

        # 전체 궤적 비교
        if len(ground_truth) > 0:
            ax1.plot(
                ground_truth[:, 0],
                ground_truth[:, 1],
                "g-",
                linewidth=3,
                label="Ground Truth",
                alpha=0.8,
            )
        if len(estimated) > 0:
            ax1.plot(
                estimated[:, 0],
                estimated[:, 1],
                "r--",
                linewidth=2,
                label="KISS-ICP",
                alpha=0.8,
            )

        ax1.set_xlabel("X (m)", fontsize=12)
        ax1.set_ylabel("Y (m)", fontsize=12)
        ax1.set_title(f"{title} - 전체 궤적", fontsize=14, fontweight='bold')
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3)
        ax1.axis("equal")

        # 오차 시각화
        if len(ground_truth) > 0 and len(estimated) > 0:
            min_len = min(len(ground_truth), len(estimated))
            errors = np.linalg.norm(
                ground_truth[:min_len, :2] - estimated[:min_len, :2], axis=1
            )

            scatter = ax2.scatter(
                ground_truth[:min_len, 0],
                ground_truth[:min_len, 1],
                c=errors,
                cmap="RdYlGn_r",
                s=30,
                alpha=0.7,
                edgecolors='black',
                linewidth=0.5
            )
            ax2.set_xlabel("X (m)", fontsize=12)
            ax2.set_ylabel("Y (m)", fontsize=12)
            ax2.set_title(f"{title} - 오차 히트맵", fontsize=14, fontweight='bold')
            ax2.axis("equal")
            ax2.grid(True, alpha=0.3)

            # 컬러바 추가
            cbar = plt.colorbar(scatter, ax=ax2)
            cbar.set_label("Position Error (m)", fontsize=11)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"2D 궤적 플롯 저장됨: {save_path}")

        return fig

    def plot_error_analysis(
        self,
        ground_truth: np.ndarray,
        estimated: np.ndarray,
        title: str = "Error Analysis",
        save_path: Optional[str] = None,
    ):
        """오차 분석 플롯"""
        if len(ground_truth) == 0 or len(estimated) == 0:
            print("오차 분석을 위한 데이터가 부족합니다.")
            return None

        min_len = min(len(ground_truth), len(estimated))
        gt = ground_truth[:min_len]
        est = estimated[:min_len]
        
        # 오차 계산
        errors = np.linalg.norm(gt - est, axis=1)
        
        # 거리 계산
        distances = np.zeros(min_len)
        for i in range(1, min_len):
            distances[i] = distances[i-1] + np.linalg.norm(gt[i] - gt[i-1])

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=self.figsize)
        
        # 1. 거리별 오차
        ax1.plot(distances, errors, 'b-', linewidth=2, alpha=0.8)
        ax1.set_xlabel('Distance (m)', fontsize=12)
        ax1.set_ylabel('Position Error (m)', fontsize=12)
        ax1.set_title('Error vs Distance', fontsize=13, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # 2. 오차 히스토그램
        ax2.hist(errors, bins=30, color='skyblue', alpha=0.7, edgecolor='black')
        ax2.axvline(np.mean(errors), color='red', linestyle='--', linewidth=2, 
                   label=f'Mean: {np.mean(errors):.4f}m')
        ax2.axvline(np.median(errors), color='green', linestyle='--', linewidth=2,
                   label=f'Median: {np.median(errors):.4f}m')
        ax2.set_xlabel('Position Error (m)', fontsize=12)
        ax2.set_ylabel('Frequency', fontsize=12)
        ax2.set_title('Error Distribution', fontsize=13, fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 시간별 오차
        ax3.plot(range(min_len), errors, 'g-', linewidth=2, alpha=0.8)
        ax3.set_xlabel('Frame Index', fontsize=12)
        ax3.set_ylabel('Position Error (m)', fontsize=12)
        ax3.set_title('Error vs Time', fontsize=13, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        # 4. 통계 정보
        ax4.axis('off')
        stats_text = f"""
        Statistics:
        ═══════════════════
        Mean Error: {np.mean(errors):.4f} m
        Median Error: {np.median(errors):.4f} m
        Std Error: {np.std(errors):.4f} m
        Max Error: {np.max(errors):.4f} m
        Min Error: {np.min(errors):.4f} m
        RMSE: {np.sqrt(np.mean(errors**2)):.4f} m
        
        Trajectory Length: {distances[-1]:.1f} m
        Number of Poses: {min_len}
        """
        ax4.text(0.1, 0.5, stats_text, fontsize=12, fontfamily='monospace',
                verticalalignment='center', bbox=dict(boxstyle="round,pad=0.5", 
                facecolor="lightgray", alpha=0.8))
        
        plt.suptitle(f'{title} - Detailed Analysis', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"오차 분석 플롯 저장됨: {save_path}")
        
        return fig

def main():
    parser = argparse.ArgumentParser(description="KISS-ICP 궤적 시각화 도구")
    parser.add_argument("--gt", type=str, required=True, help="Ground truth 궤적 파일")
    parser.add_argument("--est", type=str, required=True, help="추정 궤적 파일")
    parser.add_argument(
        "--format",
        type=str,
        default="auto",
        choices=["auto", "tum", "kitti"],
        help="파일 형식",
    )
    parser.add_argument("--output", type=str, default="./plots", help="출력 디렉토리")
    parser.add_argument(
        "--title", type=str, default="KISS-ICP Trajectory", help="플롯 제목"
    )
    parser.add_argument("--show", action="store_true", help="플롯 화면에 표시")

    args = parser.parse_args()

    # 출력 디렉토리 생성
    os.makedirs(args.output, exist_ok=True)

    visualizer = TrajectoryVisualizer()

    print(f"Ground Truth 로드 중: {args.gt}")
    ground_truth = visualizer.load_trajectory(args.gt, args.format)

    print(f"추정 궤적 로드 중: {args.est}")
    estimated = visualizer.load_trajectory(args.est, args.format)

    if len(ground_truth) == 0 and len(estimated) == 0:
        print("로드할 수 있는 데이터가 없습니다.")
        return 1

    print(f"Ground Truth 포인트 수: {len(ground_truth)}")
    print(f"추정 궤적 포인트 수: {len(estimated)}")

    # 2D 궤적 플롯
    visualizer.plot_2d_trajectory(
        ground_truth,
        estimated,
        args.title,
        os.path.join(args.output, "2d_trajectory.png"),
    )

    # 오차 분석
    visualizer.plot_error_analysis(
        ground_truth,
        estimated,
        args.title,
        os.path.join(args.output, "error_analysis.png"),
    )

    if args.show:
        plt.show()

    print(f"모든 플롯이 {args.output}에 저장되었습니다.")
    return 0

if __name__ == "__main__":
    sys.exit(main())