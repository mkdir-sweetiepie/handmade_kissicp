#!/usr/bin/env python3
"""
KISS-ICP 궤적 시각화 스크립트
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
        plt.style.use(
            "seaborn-v0_8" if "seaborn-v0_8" in plt.style.available else "seaborn"
        )

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
                "b-",
                linewidth=2,
                label="Ground Truth",
                alpha=0.8,
            )
        if len(estimated) > 0:
            ax1.plot(
                estimated[:, 0],
                estimated[:, 1],
                "r-",
                linewidth=2,
                label="Estimated",
                alpha=0.8,
            )

        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_title(f"{title} - 전체 궤적")
        ax1.legend()
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
                cmap="hot",
                s=20,
                alpha=0.7,
            )
            ax2.set_xlabel("X (m)")
            ax2.set_ylabel("Y (m)")
            ax2.set_title(f"{title} - 오차 히트맵")
            ax2.axis("equal")
            ax2.grid(True, alpha=0.3)

            # 컬러바 추가
            cbar = plt.colorbar(scatter, ax=ax2)
            cbar.set_label("Position Error (m)")

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"2D 궤적 플롯 저장됨: {save_path}")

        return fig

    def plot_3d_trajectory(
        self,
        ground_truth: np.ndarray,
        estimated: np.ndarray,
        title: str = "3D Trajectory Comparison",
        save_path: Optional[str] = None,
    ):
        """3D 궤적 비교 플롯"""
        fig = plt.figure(figsize=self.figsize)
        ax = fig.add_subplot(111, projection="3d")

        if len(ground_truth) > 0:
            ax.plot(
                ground_truth[:, 0],
                ground_truth[:, 1],
                ground_truth[:, 2],
                "b-",
                linewidth=2,
                label="Ground Truth",
                alpha=0.8,
            )

        if len(estimated) > 0:
            ax.plot(
                estimated[:, 0],
                estimated[:, 1],
                estimated[:, 2],
                "r-",
                linewidth=2,
                label="Estimated",
                alpha=0.8,
            )

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(f"{title} - 3D 궤적")
        ax.legend()

        # 시작점과 끝점 표시
        if len(ground_truth) > 0:
            ax.scatter(
                ground_truth[0, 0],
                ground_truth[0, 1],
                ground_truth[0, 2],
                c="green",
                s=100,
                marker="o",
                label="Start",
            )
            ax.scatter(
                ground_truth[-1, 0],
                ground_truth[-1, 1],
                ground_truth[-1, 2],
                c="red",
                s=100,
                marker="s",
                label="End",
            )

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"3D 궤적 플롯 저장됨: {save_path}")

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
        gt_truncated = ground_truth[:min_len]
        est_truncated = estimated[:min_len]

        # 오차 계산
        position_errors = np.linalg.norm(gt_truncated - est_truncated, axis=1)
        x_errors = np.abs(gt_truncated[:, 0] - est_truncated[:, 0])
        y_errors = np.abs(gt_truncated[:, 1] - est_truncated[:, 1])
        z_errors = (
            np.abs(gt_truncated[:, 2] - est_truncated[:, 2])
            if gt_truncated.shape[1] > 2
            else None
        )

        # 누적 거리 계산
        distances = np.zeros(min_len)
        for i in range(1, min_len):
            distances[i] = distances[i - 1] + np.linalg.norm(
                gt_truncated[i] - gt_truncated[i - 1]
            )

        # 플롯 생성
        fig, axes = plt.subplots(2, 2, figsize=self.figsize)

        # 전체 위치 오차
        axes[0, 0].plot(distances, position_errors, "r-", linewidth=1.5)
        axes[0, 0].set_xlabel("Distance (m)")
        axes[0, 0].set_ylabel("Position Error (m)")
        axes[0, 0].set_title("Position Error vs Distance")
        axes[0, 0].grid(True, alpha=0.3)

        # 축별 오차
        axes[0, 1].plot(distances, x_errors, "r-", label="X Error", alpha=0.7)
        axes[0, 1].plot(distances, y_errors, "g-", label="Y Error", alpha=0.7)
        if z_errors is not None:
            axes[0, 1].plot(distances, z_errors, "b-", label="Z Error", alpha=0.7)
        axes[0, 1].set_xlabel("Distance (m)")
        axes[0, 1].set_ylabel("Error (m)")
        axes[0, 1].set_title("Axis-wise Errors")
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # 오차 히스토그램
        axes[1, 0].hist(
            position_errors, bins=50, alpha=0.7, color="red", edgecolor="black"
        )
        axes[1, 0].set_xlabel("Position Error (m)")
        axes[1, 0].set_ylabel("Frequency")
        axes[1, 0].set_title("Error Distribution")
        axes[1, 0].grid(True, alpha=0.3)

        # 통계 정보
        mean_error = np.mean(position_errors)
        max_error = np.max(position_errors)
        rmse = np.sqrt(np.mean(position_errors**2))
        std_error = np.std(position_errors)

        stats_text = f"Statistics:\\n"
        stats_text += f"Mean Error: {mean_error:.4f} m\\n"
        stats_text += f"Max Error: {max_error:.4f} m\\n"
        stats_text += f"RMSE: {rmse:.4f} m\\n"
        stats_text += f"Std Dev: {std_error:.4f} m\\n"
        stats_text += f"Total Distance: {distances[-1]:.2f} m"

        axes[1, 1].text(
            0.1,
            0.5,
            stats_text,
            transform=axes[1, 1].transAxes,
            fontsize=12,
            verticalalignment="center",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7),
        )
        axes[1, 1].set_xlim(0, 1)
        axes[1, 1].set_ylim(0, 1)
        axes[1, 1].axis("off")
        axes[1, 1].set_title("Error Statistics")

        plt.suptitle(f"{title}", fontsize=16)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"오차 분석 플롯 저장됨: {save_path}")

        return fig

    def plot_sequence_comparison(
        self, results_dir: str, sequences: List[str], save_path: Optional[str] = None
    ):
        """여러 시퀀스의 결과 비교"""
        if not sequences:
            print("비교할 시퀀스가 없습니다.")
            return None

        fig, axes = plt.subplots(2, 2, figsize=self.figsize)

        colors = plt.cm.tab10(np.linspace(0, 1, len(sequences)))

        for i, seq in enumerate(sequences):
            seq_dir = os.path.join(results_dir, f"sequence_{seq}")
            gt_file = os.path.join(seq_dir, "ground_truth.txt")
            est_file = os.path.join(seq_dir, "estimated.txt")

            if not (os.path.exists(gt_file) and os.path.exists(est_file)):
                print(f"시퀀스 {seq}의 파일을 찾을 수 없습니다.")
                continue

            gt = self.load_trajectory(gt_file)
            est = self.load_trajectory(est_file)

            if len(gt) == 0 or len(est) == 0:
                continue

            # 시작점을 원점으로 이동
            gt_norm = gt - gt[0]
            est_norm = est - est[0]

            # 2D 궤적 플롯
            axes[0, 0].plot(
                gt_norm[:, 0],
                gt_norm[:, 1],
                color=colors[i],
                linestyle="-",
                alpha=0.8,
                linewidth=2,
                label=f"Seq {seq}",
            )

            # 오차 계산 및 플롯
            min_len = min(len(gt), len(est))
            errors = np.linalg.norm(gt[:min_len] - est[:min_len], axis=1)

            distances = np.zeros(min_len)
            for j in range(1, min_len):
                distances[j] = distances[j - 1] + np.linalg.norm(gt[j] - gt[j - 1])

            axes[0, 1].plot(
                distances,
                errors,
                color=colors[i],
                alpha=0.8,
                linewidth=2,
                label=f"Seq {seq}",
            )

            # 통계 수집
            mean_error = np.mean(errors)
            rmse = np.sqrt(np.mean(errors**2))

            axes[1, 0].bar(
                i, mean_error, color=colors[i], alpha=0.7, label=f"Seq {seq}"
            )
            axes[1, 1].bar(i, rmse, color=colors[i], alpha=0.7, label=f"Seq {seq}")

        # 플롯 설정
        axes[0, 0].set_xlabel("X (m)")
        axes[0, 0].set_ylabel("Y (m)")
        axes[0, 0].set_title("Trajectory Comparison (Normalized)")
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].axis("equal")

        axes[0, 1].set_xlabel("Distance (m)")
        axes[0, 1].set_ylabel("Position Error (m)")
        axes[0, 1].set_title("Position Error vs Distance")
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        axes[1, 0].set_xlabel("Sequence")
        axes[1, 0].set_ylabel("Mean Error (m)")
        axes[1, 0].set_title("Mean Position Error by Sequence")
        axes[1, 0].set_xticks(range(len(sequences)))
        axes[1, 0].set_xticklabels(sequences)
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].set_xlabel("Sequence")
        axes[1, 1].set_ylabel("RMSE (m)")
        axes[1, 1].set_title("RMSE by Sequence")
        axes[1, 1].set_xticks(range(len(sequences)))
        axes[1, 1].set_xticklabels(sequences)
        axes[1, 1].grid(True, alpha=0.3)

        plt.suptitle("KISS-ICP Multi-Sequence Comparison", fontsize=16)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"시퀀스 비교 플롯 저장됨: {save_path}")

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
    parser.add_argument("--sequences", type=str, nargs="*", help="비교할 시퀀스 목록")
    parser.add_argument("--results_dir", type=str, help="결과 디렉토리 (시퀀스 비교용)")

    args = parser.parse_args()

    # 출력 디렉토리 생성
    os.makedirs(args.output, exist_ok=True)

    visualizer = TrajectoryVisualizer()

    if args.results_dir and args.sequences:
        # 다중 시퀀스 비교
        fig = visualizer.plot_sequence_comparison(
            args.results_dir,
            args.sequences,
            os.path.join(args.output, "sequence_comparison.png"),
        )
    else:
        # 단일 궤적 분석
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

        # 3D 궤적 플롯 (데이터가 3D인 경우)
        if len(ground_truth) > 0 and ground_truth.shape[1] >= 3:
            visualizer.plot_3d_trajectory(
                ground_truth,
                estimated,
                args.title,
                os.path.join(args.output, "3d_trajectory.png"),
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
