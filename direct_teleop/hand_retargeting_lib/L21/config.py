# config.py
# (单位已从毫米修正为米，并且已移除Class结构)

import numpy as np

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

# ==== 机器人参数 (单位：米) ====
ROBOT_PALM_CENTER_OFFSET = np.array([-0.02184936, 0.0067269, 0.070])

FINGER_BASE_POSITIONS = {
    'thumb':  np.array([-0.0025188, -0.0050821, 0.052292]),
    'index':  np.array([-0.025865, -0.026698, 0.15421]),
    'middle': np.array([-0.030599, -0.0050984, 0.1587]),
    'ring':   np.array([-0.028294, 0.016502, 0.15421]),
    'little': np.array([-0.02467, 0.038102, 0.14521]),
}

JOINT_INFO = {
    'thumb': {
        'names': ['thumb_joint0', 'thumb_joint1', 'thumb_joint2', 'thumb_joint3', 'thumb_joint4'],
        'axes': [np.array([-1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([-1.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0])],
        'limits': [(-0.297, 0.683), (0.122, 1.78), (0.0, 0.87), (0.0, 1.29), (0.0, 1.29)],
        'joint_origins_xyz': [np.array([0.024508, -0.00083332, -0.00845]), np.array([0.010985, -0.030795, 0.012414]), np.array([-0.0010999, -0.044701, 0.034112]), np.array([-0.01277, -0.030981, 0.015322])],
        'mimic': {'thumb_joint4': {'joint': 'thumb_joint3', 'multiplier': 1.0, 'offset': 0.0}}
    },
    'index': {
        'names': ['index_joint0', 'index_joint1', 'index_joint2', 'index_joint3'],
        'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
        'joint_origins_xyz': [np.array([0.011115, -0.0047959, 0.0]), np.array([-0.0034454, 0.0010465, 0.044883]), np.array([0.0084126, -0.0012, 0.03069])],
        'mimic': {'index_joint3': {'joint': 'index_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
    },
    'middle': {
        'names': ['middle_joint0', 'middle_joint1', 'middle_joint2', 'middle_joint3'],
        'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
        'joint_origins_xyz': [np.array([0.011115, -0.0047959, 0.0]), np.array([-0.0034454, 0.0010465, 0.044883]), np.array([0.0084126, -0.0012, 0.03069])],
        'mimic': {'middle_joint3': {'joint': 'middle_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
    },
    'ring': {
        'names': ['ring_joint0', 'ring_joint1', 'ring_joint2', 'ring_joint3'],
        'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
        'joint_origins_xyz': [np.array([0.011115, -0.0047959, 0.0]), np.array([-0.0034454, 0.0010465, 0.044883]), np.array([0.0084126, -0.0012, 0.03069])],
        'mimic': {'ring_joint3': {'joint': 'ring_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
    },
    'little': {
        'names': ['little_joint0', 'little_joint1', 'little_joint2', 'little_joint3'],
        'axes': [np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(-0.26, 0.26), (0.0, 1.4), (0.0, 1.08), (0.0, 1.15)],
        'joint_origins_xyz': [np.array([0.011115, -0.0047959, 0.0]), np.array([-0.0034454, 0.0010465, 0.044883]), np.array([0.0084126, -0.0012, 0.03069])],
        'mimic': {'little_joint3': {'joint': 'little_joint2', 'multiplier': 1.06399, 'offset': 0.0}}
    },
}

# !! 确保下面这两行存在，并且没有任何缩进 !!
FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'little']
ALL_FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'little']