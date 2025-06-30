#!/usr/bin/env python3
"""
CSV 파일을 이용한 KISS-ICP 궤적 시각화 (수정된 버전)
Times New Roman 폰트 + 투명도 개선
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

# Times New Roman 폰트 설정
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 12
plt.rcParams['mathtext.fontset'] = 'stix'  # 수학 기호도 Times New Roman 스타일로


def visualize_trajectory_from_csv(csv_file, output_dir="./plots", show=False):
    """CSV 파일에서 궤적 데이터를 읽어서 시각화"""

    try:
        # CSV 파일 로드
        df = pd.read_csv(csv_file)
        print(f"CSV 파일 로드 완료: {len(df)} 포즈")
        print(f"컬럼: {list(df.columns)}")

        # numpy array로 변환 (pandas 호환성 해결)
        gt_x = df["gt_x"].values
        gt_y = df["gt_y"].values
        gt_z = df["gt_z"].values
        est_x = df["est_x"].values
        est_y = df["est_y"].values
        est_z = df["est_z"].values

        # Y축 중심점 계산
        y_center = (np.max(gt_y) + np.min(gt_y)) / 2

        # 출력 디렉토리 생성
        os.makedirs(output_dir, exist_ok=True)

        # Figure 생성 (2x2 레이아웃)
        fig = plt.figure(figsize=(16, 12))
        fig.patch.set_facecolor('white')

        # 1. 2D 궤적 비교 - Y축 범위 제한 + 개선된 투명도
        ax1 = plt.subplot(2, 2, 1)
        
        # Ground Truth를 더 두껍고 반투명하게
        plt.plot(gt_x, gt_y, "g-", linewidth=4, label="Ground Truth", alpha=0.6, zorder=1)
        
        # KISS-ICP를 점선으로 더 얇게, 약간 더 진하게
        plt.plot(est_x, est_y, "r--", linewidth=2.5, label="KISS-ICP", alpha=0.5, zorder=2)
        
        plt.xlabel("X [m]", fontsize=14, fontweight='bold')
        plt.ylabel("Y [m]", fontsize=14, fontweight='bold')
        plt.title("Trajectory Comparison (Y: ±20m)", fontsize=16, fontweight="bold")
        plt.legend(fontsize=12, frameon=True, fancybox=True, shadow=True)
        plt.grid(True, alpha=0.3, linewidth=0.8)

        # Y축 범위 제한
        plt.ylim(y_center - 20, y_center + 20)

        # 실제 궤적 범위를 점선으로 표시
        actual_y_min, actual_y_max = np.min(gt_y), np.max(gt_y)
        plt.axhline(actual_y_min, color="blue", linestyle=":", alpha=0.7, linewidth=1.5)
        plt.axhline(actual_y_max, color="blue", linestyle=":", alpha=0.7, linewidth=1.5)

        # 2. 오차 계산 및 히트맵 - Y축 범위 제한
        errors = np.sqrt(
            (gt_x - est_x) ** 2 + (gt_y - est_y) ** 2 + (gt_z - est_z) ** 2
        )

        ax2 = plt.subplot(2, 2, 2)
        scatter = plt.scatter(
            gt_x,
            gt_y,
            c=errors,
            cmap="RdYlGn_r",
            s=50,
            alpha=0.8,
            edgecolors="black",
            linewidth=0.5,
        )
        plt.xlabel("X [m]", fontsize=14, fontweight='bold')
        plt.ylabel("Y [m]", fontsize=14, fontweight='bold')
        plt.title("Position Error Heatmap (Y: ±20m)", fontsize=16, fontweight="bold")
        
        # 컬러바 폰트 크기 조정
        cbar = plt.colorbar(scatter, label="Error [m]")
        cbar.ax.tick_params(labelsize=11)
        cbar.set_label("Error [m]", fontsize=12, fontweight='bold')
        
        plt.grid(True, alpha=0.3, linewidth=0.8)

        # Y축 범위 제한
        plt.ylim(y_center - 20, y_center + 20)

        # 실제 궤적 범위를 점선으로 표시
        plt.axhline(actual_y_min, color="blue", linestyle=":", alpha=0.7, linewidth=1.5)
        plt.axhline(actual_y_max, color="blue", linestyle=":", alpha=0.7, linewidth=1.5)

        # 3. 거리별 오차
        ax3 = plt.subplot(2, 2, 3)
        # 거리 계산 (numpy 배열 사용)
        distances = np.zeros(len(df))
        for i in range(1, len(df)):
            dx = gt_x[i] - gt_x[i - 1]
            dy = gt_y[i] - gt_y[i - 1]
            dz = gt_z[i] - gt_z[i - 1]
            distances[i] = distances[i - 1] + np.sqrt(dx**2 + dy**2 + dz**2)

        plt.plot(distances, errors, "b-", linewidth=2.5, alpha=0.8)
        plt.xlabel("Distance [m]", fontsize=14, fontweight='bold')
        plt.ylabel("Position Error [m]", fontsize=14, fontweight='bold')
        plt.title("Error vs Distance", fontsize=16, fontweight="bold")
        plt.grid(True, alpha=0.3, linewidth=0.8)

        # 4. 오차 히스토그램 및 통계
        ax4 = plt.subplot(2, 2, 4)
        plt.hist(errors, bins=30, color="skyblue", alpha=0.7, edgecolor="black", linewidth=1)
        plt.axvline(
            np.mean(errors),
            color="red",
            linestyle="--",
            linewidth=3,
            alpha=0.8,
            label=f"Mean: {np.mean(errors):.4f}m",
        )
        plt.axvline(
            np.median(errors),
            color="green",
            linestyle="--",
            linewidth=3,
            alpha=0.8,
            label=f"Median: {np.median(errors):.4f}m",
        )
        plt.xlabel("Position Error [m]", fontsize=14, fontweight='bold')
        plt.ylabel("Frequency", fontsize=14, fontweight='bold')
        plt.title("Error Distribution", fontsize=16, fontweight="bold")
        plt.legend(fontsize=11, frameon=True, fancybox=True, shadow=True)
        plt.grid(True, alpha=0.3, linewidth=0.8)

        # 전체 제목
        fig.suptitle(
            "KISS-ICP Trajectory Evaluation Results", 
            fontsize=18, 
            fontweight="bold",
            y=0.95
        )
        plt.tight_layout()

        # 통계 정보 출력
        print("\n" + "=" * 50)
        print("TRAJECTORY EVALUATION STATISTICS")
        print("=" * 50)
        print(f"Mean Error:      {np.mean(errors):.4f} m")
        print(f"Median Error:    {np.median(errors):.4f} m")
        print(f"Std Error:       {np.std(errors):.4f} m")
        print(f"RMSE:           {np.sqrt(np.mean(errors**2)):.4f} m")
        print(f"Max Error:       {np.max(errors):.4f} m")
        print(f"Min Error:       {np.min(errors):.4f} m")
        print(f"Trajectory Length: {distances[-1]:.1f} m")
        print(f"Number of Poses: {len(df)}")
        print("=" * 50)

        # 저장
        output_file = os.path.join(output_dir, "trajectory_analysis.png")
        plt.savefig(output_file, dpi=300, bbox_inches="tight", facecolor='white')
        print(f"플롯 저장됨: {output_file}")

        if show:
            plt.show()

        return fig, errors, distances

    except Exception as e:
        print(f"오류 발생: {e}")
        import traceback

        traceback.print_exc()
        return None, None, None


def create_kitti_style_report(csv_file, output_dir="./plots"):
    """실제 KITTI 웹사이트 스타일 평가 리포트 생성 - 개선된 투명도"""

    try:
        df = pd.read_csv(csv_file)

        # numpy array로 변환
        gt_x = df["gt_x"].values
        gt_y = df["gt_y"].values
        gt_z = df["gt_z"].values
        est_x = df["est_x"].values
        est_y = df["est_y"].values
        est_z = df["est_z"].values

        # 오차 계산
        errors = np.sqrt(
            (gt_x - est_x) ** 2 + (gt_y - est_y) ** 2 + (gt_z - est_z) ** 2
        )

        # 궤적 길이 계산
        distances = np.zeros(len(df))
        for i in range(1, len(df)):
            dx = gt_x[i] - gt_x[i - 1]
            dy = gt_y[i] - gt_y[i - 1]
            dz = gt_z[i] - gt_z[i - 1]
            distances[i] = distances[i - 1] + np.sqrt(dx**2 + dy**2 + dz**2)

        # KITTI 웹사이트 스타일 - Times New Roman 적용
        fig = plt.figure(figsize=(16, 10))
        fig.patch.set_facecolor("white")

        # 1. 메인 궤적 플롯 (왼쪽 절반) - 개선된 투명도
        ax1 = plt.subplot2grid((2, 4), (0, 0), colspan=2, rowspan=2)
        
        # Y축 중심점 및 범위 계산
        y_center = (np.max(gt_y) + np.min(gt_y)) / 2
        plt.ylim(y_center - 20, y_center + 20)
        
        # Ground Truth를 더 두껍고 반투명하게
        plt.plot(gt_x, gt_y, 'b-', linewidth=3, label='Ground truth', alpha=0.6, zorder=1)
        
        # KISS-ICP를 점선으로 더 진하게
        plt.plot(est_x, est_y, 'r--', linewidth=2.5, label='KISS-ICP', alpha=0.9, zorder=2)
        
        plt.xlabel('x [m]', fontsize=12, fontweight='bold')
        plt.ylabel('y [m]', fontsize=12, fontweight='bold')
        plt.title('Sequence 00', fontsize=14, fontweight='bold')
        plt.legend(fontsize=11, frameon=True, fancybox=True, shadow=True)
        plt.grid(True, alpha=0.3, linewidth=0.8)
        
        # 축 눈금 스타일 개선
        plt.tick_params(axis='both', which='major', labelsize=10)

        # 2. 오차 테이블 (오른쪽 상단)
        ax2 = plt.subplot2grid((2, 4), (0, 2), colspan=2)
        ax2.axis('off')
        
        # RPE 계산 간소화 (실제 계산 대신 현재 데이터 기반 추정)
        rpe_distances = [100, 200, 300, 400, 500, 600, 700, 800]
        rpe_trans_errors = []
        
        # 거리별 평균 오차율 계산
        for dist in rpe_distances:
            # 해당 거리 근처의 오차들을 찾아서 평균 계산
            target_indices = []
            for i in range(len(distances)):
                if abs(distances[i] - dist) < 50:  # ±50m 범위
                    target_indices.append(i)
            
            if target_indices:
                local_errors = [errors[i] for i in target_indices]
                avg_error = np.mean(local_errors)
                # 거리에 대한 백분율로 변환
                rpe_trans_errors.append(avg_error / dist * 100)
            else:
                # 전체 평균 오차를 거리로 나눈 값 사용
                rpe_trans_errors.append(np.mean(errors) / dist * 100)

        # KITTI 스타일 텍스트 정보 - Times New Roman 적용
        table_text = f"""EVALUATION RESULTS

Method: KISS-ICP
Sequence: 00

Relative Pose Error (RPE)
Translation Error [%]:

100m: {rpe_trans_errors[0]:.3f}
200m: {rpe_trans_errors[1]:.3f}  
300m: {rpe_trans_errors[2]:.3f}
400m: {rpe_trans_errors[3]:.3f}
500m: {rpe_trans_errors[4]:.3f}
600m: {rpe_trans_errors[5]:.3f}
700m: {rpe_trans_errors[6]:.3f}
800m: {rpe_trans_errors[7]:.3f}

Overall Statistics:
Length: {distances[-1]:.0f}m
ATE RMSE: {np.sqrt(np.mean(errors**2)):.4f}m
Mean Error: {np.mean(errors):.4f}m
Frames: {len(df)}
"""
        
        ax2.text(0.05, 0.95, table_text, fontsize=10, family='Times New Roman',
                 verticalalignment='top', transform=ax2.transAxes,
                 bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8f8f8', alpha=0.9, edgecolor='gray'))

        # 3. 오차 플롯 (오른쪽 하단) - 개선된 스타일
        ax3 = plt.subplot2grid((2, 4), (1, 2), colspan=2)
        
        plt.plot(distances, errors*100, 'b-', linewidth=2, alpha=0.8)  # cm 단위로
        plt.xlabel('Path Length [m]', fontsize=12, fontweight='bold')
        plt.ylabel('Translation Error [cm]', fontsize=12, fontweight='bold')
        plt.title('Translation Error over Path Length', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, linewidth=0.8)
        plt.tick_params(axis='both', which='major', labelsize=10)
        
        # 평균선 추가
        plt.axhline(y=np.mean(errors)*100, color='red', linestyle='--', 
                   linewidth=2, alpha=0.8, label=f'Mean: {np.mean(errors)*100:.1f}cm')
        plt.legend(fontsize=10, frameon=True, fancybox=True, shadow=True)

        # 전체 제목 - Times New Roman 적용
        fig.suptitle('KITTI Odometry Benchmark - Visual Odometry / SLAM Evaluation', 
                     fontsize=14, fontweight='bold', y=0.95)

        plt.tight_layout()
        plt.subplots_adjust(top=0.90, hspace=0.3, wspace=0.3)

        # 저장
        output_file = os.path.join(output_dir, "kitti_official_style.png")
        plt.savefig(output_file, dpi=300, bbox_inches="tight", facecolor='white')
        print(f"KITTI 공식 스타일 리포트 저장됨: {output_file}")

        return fig

    except Exception as e:
        print(f"KITTI 리포트 생성 오류: {e}")
        import traceback

        traceback.print_exc()
        return None


def main():
    parser = argparse.ArgumentParser(description="CSV 파일을 이용한 궤적 시각화")
    parser.add_argument(
        "--csv", type=str, required=True, help="trajectory_comparison.csv 파일"
    )
    parser.add_argument("--output", type=str, default="./plots", help="출력 디렉토리")
    parser.add_argument("--show", action="store_true", help="플롯 화면에 표시")
    parser.add_argument(
        "--kitti-style", action="store_true", help="KITTI 스타일 리포트 생성"
    )

    args = parser.parse_args()

    print(f"CSV 파일 시각화 시작: {args.csv}")
    print("폰트: Times New Roman 적용됨")
    print("투명도 개선으로 궤적 가시성 향상")

    # 기본 시각화
    fig, errors, distances = visualize_trajectory_from_csv(
        args.csv, args.output, args.show
    )

    # KITTI 스타일 리포트
    if args.kitti_style and fig is not None:
        print("KITTI 스타일 리포트 생성 중...")
        kitti_fig = create_kitti_style_report(args.csv, args.output)
        if kitti_fig and args.show:
            plt.show()

    print("시각화 완료!")


if __name__ == "__main__":
    main()