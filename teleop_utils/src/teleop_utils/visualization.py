# visualization.py

import numpy as np
import matplotlib.pyplot as plt
import json
import argparse  # <--- 新增
from config import Config
from utils_1 import unity_lhs_to_python_rhs

COLOR_MAP = {'thumb': 'r', 'index': 'g', 'middle': 'b', 'ring': 'y', 'little': 'c'}

# ... (plot_hand 函数保持不变) ...
def plot_hand(ax, keypoints, title, config):
    # 绘制手指
    for finger in config.FINGER_NAMES:
        mcp_idx = config.KEYPOINT_MAP[f'{finger}_mcp']
        pip_idx = config.KEYPOINT_MAP[f'{finger}_pip']
        dip_idx = config.KEYPOINT_MAP[f'{finger}_dip']
        tip_idx = config.KEYPOINT_MAP[f'{finger}_tip']

        # 检查所有关节点是否存在
        if all(idx < len(keypoints) for idx in [mcp_idx, pip_idx, dip_idx, tip_idx]):
            points_indices = [mcp_idx, pip_idx, dip_idx, tip_idx]
            finger_points = keypoints[points_indices]
            ax.plot(finger_points[:, 0], finger_points[:, 1], finger_points[:, 2],
                    marker='o', color=COLOR_MAP[finger], label=finger.capitalize())

    # 绘制手掌
    wrist_idx = config.KEYPOINT_MAP['wrist']
    palm_center_idx = config.KEYPOINT_MAP['palm_center']

    if wrist_idx < len(keypoints) and palm_center_idx < len(keypoints):
        wrist = keypoints[wrist_idx]
        palm_center = keypoints[palm_center_idx]
        mcp_indices = [config.KEYPOINT_MAP[f'{f}_mcp'] for f in config.FINGER_NAMES]

        if all(idx < len(keypoints) for idx in mcp_indices):
            mcp_points = keypoints[mcp_indices]
            ax.scatter(wrist[0], wrist[1], wrist[2], c='k', marker='s', s=100, label='Wrist')
            ax.scatter(palm_center[0], palm_center[1], palm_center[2], c='k', marker='^', s=100, label='Palm Center')
            ax.plot([wrist[0], palm_center[0]], [wrist[1], palm_center[1]], [wrist[2], palm_center[2]], 'k--')
            for mcp in mcp_points:
                ax.plot([palm_center[0], mcp[0]], [palm_center[1], mcp[1]], [palm_center[2], mcp[2]], 'k:')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    ax.view_init(elev=30, azim=-60)
    ax.grid(True)
    ax.set_aspect('equal', 'box')


def main():
    # --- [核心修复] 添加命令行参数解析 ---
    parser = argparse.ArgumentParser(description="可视化重定向结果")
    parser.add_argument('--normalize', action='store_true',
                        help="启用此开关可将'w'图归一化(平移和缩放)以方便观察。")
    args = parser.parse_args()
    # --- 修复结束 ---

    config = Config()
    filepath = 'debug_output.json'

    try:
        with open(filepath, 'r') as f:
            all_frames_data = json.load(f)
    except FileNotFoundError:
        print(f"错误: '{filepath}' 未找到。请先运行 debug_from_pico_sample.py")
        return

    for i, frame_data in enumerate(all_frames_data):
        w_lhs = np.array(frame_data['w_lhs'])
        v_rhs = np.array(frame_data['v_rhs'])
        fk_rhs = np.array(frame_data['fk_rhs'])

        # 将w从LHS转换为RHS
        w_rhs_to_plot = unity_lhs_to_python_rhs(w_lhs)
        w_plot_title = "Human Hand (w) in RHS"

        # --- [核心修复] 条件化的归一化逻辑 ---
        if args.normalize:
            print("可视化归一化已启用。")
            w_plot_title = "Human Hand (w) [Normalized for Viz]"

            # 1. 计算中心点（以手腕为基准）
            w_wrist_pos = w_rhs_to_plot[config.KEYPOINT_MAP['wrist']]

            # 2. 将手平移到原点
            w_rhs_to_plot = w_rhs_to_plot - w_wrist_pos

            # 3. 读取并应用尺度因子
            scale_factor = frame_data.get('scale_factor', 1.0)
            if scale_factor != 1.0:
                print(f"应用尺度因子: {scale_factor:.3f}")
                w_rhs_to_plot *= scale_factor
        # --- 修复结束 ---

        fig = plt.figure(figsize=(18, 6))
        fig.suptitle(f"Frame {i + 1}/{len(all_frames_data)}", fontsize=16)

        ax1 = fig.add_subplot(131, projection='3d')
        plot_hand(ax1, w_rhs_to_plot, w_plot_title, config)

        ax2 = fig.add_subplot(132, projection='3d')
        plot_hand(ax2, v_rhs, "Target (v) in RHS", config)

        ax3 = fig.add_subplot(133, projection='3d')
        plot_hand(ax3, fk_rhs, "Robot FK (fk) in RHS", config)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()


if __name__ == "__main__":
    main()