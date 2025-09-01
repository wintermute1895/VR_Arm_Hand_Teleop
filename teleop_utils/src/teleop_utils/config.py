# config.py (Auto-generated from URDF)

import numpy as np

class Config:
    # --- 权重系数 ---

    HAND_MODEL = "L21"

    LAMBDA_ALIGNMENT = 5.0
    LAMBDA_SMOOTHNESS = 0.1

    # 优化器参数
    MAX_ITER = 100
    TOLERANCE = 1e-5

    # ==== 权威关键点定义 ====
    KEYPOINT_MAP = {
        'wrist': 0,
        'thumb_mcp': 1, 'thumb_pip': 2, 'thumb_dip': 3, 'thumb_tip': 4,
        'index_mcp': 5, 'index_pip': 6, 'index_dip': 7, 'index_tip': 8,
        'middle_mcp': 9, 'middle_pip': 10, 'middle_dip': 11, 'middle_tip': 12,
        'ring_mcp': 13, 'ring_pip': 14, 'ring_dip': 15, 'ring_tip': 16,
        'little_mcp': 17, 'little_pip': 18, 'little_dip': 19, 'little_tip': 20,
        'palm_center': 21
    }

    # ==== [最终版] 机器人参数 - 从URDF自动生成 ====
    ROBOT_PALM_CENTER_OFFSET = np.array([-21.84936, 6.7269, 70.0])

    FINGER_BASE_POSITIONS = {
        'thumb': np.array([-2.5188, -5.0821, 52.292]),
        'index': np.array([-25.865, -26.698, 154.21]),
        'middle': np.array([-30.599, -5.0984, 158.7]),
        'ring': np.array([-28.294, 16.502, 154.21]),
        'little': np.array([-24.67, 38.102, 145.21]),
    }

    JOINT_INFO = {
        'thumb': {
            'names': ['thumb_joint0', 'thumb_joint1', 'thumb_joint2', 'thumb_joint3', 'thumb_joint4'],
            'axes': [np.array([-1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([-1.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0])],
            'limits': [(-0.297, 0.683), (0.122, 1.78), (0.0, 0.87), (0.0, 1.29), (0.0, 1.29)],
            'joint_origins_xyz': [np.array([24.508, -0.83332, -8.45]), np.array([10.985, -30.795, 12.414]), np.array([-1.0999, -44.701, 34.112]), np.array([-12.77, -30.981, 15.322])],
            'mimic': {'thumb_joint4': {'joint': 'thumb_joint3', 'multiplier': 1.0, 'offset': 0.0}}
        },
        'index': {
            'names': ['index_joint0', 'index_joint1', 'index_joint2', 'index_joint3'],
            'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
            'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
            'joint_origins_xyz': [np.array([11.115, -4.7959, 0.0]), np.array([-3.4454, 1.0465, 44.883]), np.array([8.4126, -1.2, 30.69])],
            'mimic': {'index_joint3': {'joint': 'index_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
        },
        'middle': {
            'names': ['middle_joint0', 'middle_joint1', 'middle_joint2', 'middle_joint3'],
            'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
            'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
            'joint_origins_xyz': [np.array([11.115, -4.7959, 0.0]), np.array([-3.4454, 1.0465, 44.883]), np.array([8.4126, -1.2, 30.69])],
            'mimic': {'middle_joint3': {'joint': 'middle_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
        },
        'ring': {
            'names': ['ring_joint0', 'ring_joint1', 'ring_joint2', 'ring_joint3'],
            'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
            'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
            'joint_origins_xyz': [np.array([11.115, -4.7959, 0.0]), np.array([-3.4454, 1.0465, 44.883]), np.array([8.4126, -1.2, 30.69])],
            'mimic': {'ring_joint3': {'joint': 'ring_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
        },
        'little': {
            'names': ['little_joint0', 'little_joint1', 'little_joint2', 'little_joint3'],
            'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
            'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
            'joint_origins_xyz': [np.array([11.115, -4.7959, 0.0]), np.array([-3.4454, 1.0465, 44.883]), np.array([8.4126, -1.2, 30.69])],
            'mimic': {'little_joint3': {'joint': 'little_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
        },
    }

    FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'little']
    ALL_FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'little']