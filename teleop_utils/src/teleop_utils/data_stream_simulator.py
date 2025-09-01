# data_stream_simulator.py

import socket
import json
import time
import numpy as np
import scipy

# 我们需要Config和PICODataHandler来进行初始的数据转换
from config import Config
from pico_data_handler import PICODataHandler
from scipy.spatial.transform import Rotation


def vec_to_rot_matrix(v1, v2):
    """
    [已修复]
    计算将向量v1旋转到v2的旋转矩阵，并处理所有边界情况。
    """
    # 归一化输入向量
    v1_u = v1 / (np.linalg.norm(v1) + 1e-8)
    v2_u = v2 / (np.linalg.norm(v2) + 1e-8)

    # 计算叉积和点积
    v = np.cross(v1_u, v2_u)
    c = np.dot(v1_u, v2_u)

    # 边界情况 1: 向量已经对齐 (v1 和 v2 同向)
    if abs(c - 1.0) < 1e-8:
        return np.eye(3) # 无需旋转，返回单位矩阵

    # 边界情况 2: 向量完全相反 (v1 和 v2 反向)
    if abs(c + 1.0) < 1e-8:
        # 需要一个180度的旋转。旋转轴可以是任何与v1垂直的向量。
        # 我们找一个简单的轴。
        axis = np.array([0.0, 0.0, 1.0])
        if np.linalg.norm(np.cross(v1_u, axis)) < 1e-8: # 如果v1也和z轴平行
            axis = np.array([0.0, 1.0, 0.0]) # 换一个轴
        return Rotation.from_rotvec(np.pi * np.cross(v1_u, axis)).as_matrix()

    # 标准情况 (Rodrigues' rotation formula)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + kmat + kmat @ kmat * ((1 - c) / (s ** 2))
class HandSkeleton:
    """一个简化的手部骨骼模型，用于IK和FK"""

    def __init__(self, w_lhs_initial, config):
        self.config = config
        self.keypoint_map = config.KEYPOINT_MAP

        # 存储骨骼的初始状态
        self.base_positions = w_lhs_initial.copy()
        self.bone_vectors = {}
        self.bone_lengths = {}

        # 计算初始骨骼向量和长度
        for finger in config.FINGER_NAMES:
            parts = ['mcp', 'pip', 'dip', 'tip']
            for i in range(len(parts) - 1):
                start_name = f"{finger}_{parts[i]}"
                end_name = f"{finger}_{parts[i + 1]}"
                start_idx = self.keypoint_map[start_name]
                end_idx = self.keypoint_map[end_name]

                vec = self.base_positions[end_idx] - self.base_positions[start_idx]
                self.bone_vectors[(start_name, end_name)] = vec
                self.bone_lengths[(start_name, end_name)] = np.linalg.norm(vec)

    def ik_simple(self, w_lhs):
        """简化的IK：从一个姿态计算关节旋转"""
        rotations = {}
        for finger in self.config.FINGER_NAMES:
            parts = ['mcp', 'pip', 'dip']
            for i in range(len(parts)):
                start_name = f"{finger}_{parts[i]}"
                end_name = f"{finger}_{parts[i + 1]}" if i < len(parts) - 1 else f"{finger}_tip"

                start_idx = self.keypoint_map[start_name]
                end_idx = self.keypoint_map[end_name]

                base_vec = self.bone_vectors.get((start_name, end_name))
                current_vec = w_lhs[end_idx] - w_lhs[start_idx]

                if base_vec is not None and np.linalg.norm(base_vec) > 1e-6 and np.linalg.norm(current_vec) > 1e-6:
                    rotations[start_name] = vec_to_rot_matrix(base_vec, current_vec)
                else:
                    rotations[start_name] = np.eye(3)
        return rotations

    def fk(self, root_positions, joint_rotations):
        """FK：根据关节旋转计算新的关键点位置"""
        new_positions = root_positions.copy()

        for finger in self.config.FINGER_NAMES:
            parts = ['mcp', 'pip', 'dip', 'tip']
            current_transform = np.eye(4)  # Not used here, simplified

            for i in range(len(parts) - 1):
                start_name = f"{finger}_{parts[i]}"
                end_name = f"{finger}_{parts[i + 1]}"
                start_idx = self.keypoint_map[start_name]
                end_idx = self.keypoint_map[end_name]

                # 获取旋转
                rot_mat = joint_rotations.get(start_name, np.eye(3))

                # 计算旋转后的骨骼向量
                base_vec = self.bone_vectors[(start_name, end_name)]
                rotated_vec = rot_mat @ base_vec

                # 计算终点位置
                new_positions[end_idx] = new_positions[start_idx] + rotated_vec

        return new_positions


def main():
    config = Config()
    pico_handler = PICODataHandler()

    # --- 1. 加载并处理一次PICO样本，作为我们的“静止”基础姿态 ---
    try:
        with open('pico_sample_data.json', 'r') as f:
            base_frame_data = json.load(f)
        w_lhs_base = pico_handler.process_pico_data(base_frame_data)
    except FileNotFoundError:
        print("错误: 'pico_sample_data.json' 未找到。")
        return

    # --- 2. 初始化骨骼模型和IK/FK状态 ---
    skeleton = HandSkeleton(w_lhs_base, config)
    # 假设初始姿态就是我们的“零”旋转
    base_rotations = {name: np.eye(3) for name in skeleton.ik_simple(w_lhs_base).keys()}

    # 初始化随机游走的状态（现在是在旋转空间）
    current_rotations = base_rotations.copy()
    angular_velocities = {name: np.zeros(3) for name in base_rotations.keys()}  # 旋转向量作为速度

    # --- 模拟参数 ---
    DAMPING = 0.90
    ACCELERATION_SCALE = 0.05  # 扰动幅度
    RETURN_STRENGTH = 0.1

    # --- 网络设置 ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    TARGET_IP = "127.0.0.1";
    TARGET_PORT = 9999
    print(f"Kinematic Simulator started. Sending to {TARGET_IP}:{TARGET_PORT}...")

    try:
        while True:
            # --- 3. 在关节旋转空间进行平滑随机游走 ---
            next_rotations = {}
            for name, rot in current_rotations.items():
                # a. 计算随机加速度和回正力
                random_acc = np.random.randn(3) * ACCELERATION_SCALE
                # 简单的回正力，让旋转回到初始姿态
                return_force = -Rotation.from_matrix(rot).as_rotvec() * RETURN_STRENGTH

                # b. 更新角速度
                angular_velocities[name] = angular_velocities[name] * DAMPING + random_acc + return_force

                # c. 应用速度更新旋转
                delta_rotation = Rotation.from_rotvec(angular_velocities[name])
                next_rotations[name] = (delta_rotation * Rotation.from_matrix(rot)).as_matrix()

            current_rotations = next_rotations

            # --- 4. 通过FK计算当前帧的三维坐标 ---
            # 我们只移动手指，保持手掌和手腕不动
            root_positions = w_lhs_base.copy()
            current_w_lhs = skeleton.fk(root_positions, current_rotations)

            # --- 5. 组装并发送数据包 ---
            joints_list = []
            for i in range(len(current_w_lhs)):
                joint_dict = {
                    "ID": i,
                    "Pos": current_w_lhs[i].tolist(),
                    "Rot": [0.0, 0.0, 0.0, 1.0]
                }
                joints_list.append(joint_dict)

            packet = {"hand": "HandLeft", "isActive": True, "joints": joints_list}
            sock.sendto(json.dumps(packet).encode('utf-8'), (TARGET_IP, TARGET_PORT))

            print("Sent kinematically valid frame...", end='\r')
            time.sleep(1 / 60)

    except KeyboardInterrupt:
        print("\nSimulator stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()