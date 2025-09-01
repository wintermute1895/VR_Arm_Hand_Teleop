#!/usr/bin/env python3
import numpy as np
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices

# 辅助函数: 计算齐次变换矩阵的逆
def inverse_matrix(matrix):
    """计算4x4齐次变换矩阵的逆矩阵"""
    R = matrix[:3, :3]
    t = matrix[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

# === STEP 1: 从您获取的数据中粘贴 ===

# PICO手腕位姿 (右手系)
pico_pos = [0.09154903888702393, 1.1909642219543457, 0.5340496301651001]
pico_quat = [-0.1071423664689064, 0.37787777185440063, -0.37983348965644836, 0.8375294208526611] # (qx, qy, qz, qw)

# 机器人Link6位姿 (右手系)
robot_pos = [-0.22100230, 0.08940983, 0.46021561]
robot_quat = [-0.31810205, 0.71540731, 0.31759992, -0.53491472] # (qx, qy, qz, qw)


# === STEP 2: 将位姿转换为 4x4 齐次矩阵 ===
# 机器人位姿矩阵 T_world_robot
T_robot = quaternion_matrix(robot_quat)
T_robot[:3, 3] = robot_pos

# PICO位姿矩阵 T_world_pico
T_pico = quaternion_matrix(pico_quat)
T_pico[:3, 3] = pico_pos


# === STEP 3: 计算变换矩阵 T_pico_to_robot ===
# 我们要找的变换 T_pico_to_robot 满足: T_robot = T_pico_to_robot * T_pico
# 因此: T_pico_to_robot = T_robot * inverse(T_pico)

T_pico_inv = inverse_matrix(T_pico)
T_pico_to_robot = T_robot @ T_pico_inv


# === STEP 4: 打印结果，用于粘贴到主控制器脚本 ===
print("="*60)
print("标定完成！请将这个NumPy数组完整复制到您的控制器脚本中：")
print("="*60)
print("self.T_pico_to_robot = np.array([")
for row in T_pico_to_robot:
    print(f"    [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}, {row[3]:.8f}],")
print("])")
print("="*60)