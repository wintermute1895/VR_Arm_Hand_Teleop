# config_l10.py
# (根据 linkerhand_l10_7_left.urdf 和原始代码结构精确生成)

import numpy as np

# --- 权重系数 (从原始代码模板继承，需要您根据实际效果调试) ---
LAMBDA_ALIGNMENT = 170.0
LAMBDA_COORDINATION = 2.0  # 原始代码中的耦合项，建议从一个较小的值开始
LAMBDA_SMOOTHNESS = 0.0002

# 优化器参数
MAX_ITER = 8
TOLERANCE = 1e-3

# Sigmoid 函数参数
SIGMOID_K = 10.0
SIGMOID_C = 0.5

# =================================================================
# ==== PICO 数据映射 (数据适配层) ====
# =================================================================
# 将PICO的ID号，映射到我们内部使用的逻辑名称
PICO_ID_TO_LOGICAL_NAME = {
    1: 'wrist',
    # 拇指
    2: 'thumb_mcp', 3: 'thumb_proximal', 4: 'thumb_distal', 5: 'thumb_tip',
    # 食指
    6: 'index_mcp', 7: 'index_proximal', 8: 'index_middle', 9: 'index_distal', 10: 'index_tip',
    # 中指
    11: 'middle_mcp', 12: 'middle_proximal', 13: 'middle_middle', 14: 'middle_distal', 15: 'middle_tip',
    # 无名指
    16: 'ring_mcp', 17: 'ring_proximal', 18: 'ring_middle', 19: 'ring_distal', 20: 'ring_tip',
    # 小指
    21: 'pinky_mcp', 22: 'pinky_proximal', 23: 'pinky_middle', 24: 'pinky_distal', 25: 'pinky_tip',
}
# 这个字典的键必须和原始代码中使用的手指名 ('thumb', 'index', etc.) 完全一致
PICO_FINGER_KEYPOINT_IDS = {
    'thumb': [2, 3, 4, 5],
    'index': [6, 7, 8, 9, 10],
    'middle': [11, 12, 13, 14, 15],
    'ring': [16, 17, 18, 19, 20],
    'pinky': [21, 22, 23, 24, 25],
}
PICO_WRIST_ID = 1

# =================================================================
# ==== L10 URDF 运动学参数 (单位：米) ====
# =================================================================

# 在算法中使用的手指名称
FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']
ALL_FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']

# 独立驱动的关节数量 (用于创建优化向量)
INDEPENDENT_JOINTS_PER_FINGER = {
    'thumb': 3, 'index': 2, 'middle': 1, 'ring': 2, 'pinky': 2
}

# 每个手指基座关节相对于 hand_base_link 的位置
FINGER_BASE_POSITIONS = {
    'thumb':  np.array([-0.013419, -0.012551, 0.060602]),
    'index':  np.array([-0.0021643, -0.026654, 0.13253]),
    'middle': np.array([-0.0021316, -0.0076542, 0.15281]),
    'ring':   np.array([-0.0021643, 0.011346, 0.13253]),
    'pinky':  np.array([-0.00012074, 0.030346, 0.12755])
}

JOINT_INFO = {
    'thumb': {
        'names': ['thumb_cmc_roll', 'thumb_cmc_yaw', 'thumb_cmc_pitch', 'thumb_mcp', 'thumb_ip'],
        'axes': [np.array(x) for x in [[-0.99996, 0, -0.0087265], [-0.008517, -0.21782, 0.97595], [0, 1, 0], [0, 1, 0], [0, 1, 0]]],
        'limits': [(0, 1.1339), (0, 1.9189), (0, 0.5149), (0, 0.7152), (0, 0.7763)],
        'offsets': [
            np.array([0.035797, 0.00065879, 0.00045944]), # thumb_cmc_roll -> thumb_cmc_yaw
            np.array([0.0046051, -0.014383, -0.0051478]),# thumb_cmc_yaw -> thumb_cmc_pitch
            np.array([0.0061722, 0, 0.047968]),         # thumb_cmc_pitch -> thumb_mcp
            np.array([-0.00017064, 0, 0.038665]),        # thumb_mcp -> thumb_ip
            np.array([0, 0, 0.025]) # 估算的指尖偏移 (distal -> tip)
        ],
        'mimic': {
            'thumb_mcp': {'joint': 'thumb_cmc_pitch', 'multiplier': 1.3898, 'offset': 0.0},
            'thumb_ip': {'joint': 'thumb_cmc_pitch', 'multiplier': 1.508, 'offset': 0.0}
        }
    },
    'index': {
        'names': ['index_mcp_roll', 'index_mcp_pitch', 'index_pip', 'index_dip'],
        'axes': [np.array(x) for x in [[0.99996, 0, 0.0087265], [0, 1, 0], [0, 1, 0], [0, 1, 0]]],
        'limits': [(0, 0.2181), (0, 1.3607), (0, 1.8317), (0, 0.628)],
        'offsets': [
            np.array([0.0020763, 0, 0.015294]),
            np.array([-0.0013807, 0, 0.035624]),
            np.array([-0.0054686, 0, 0.025665]),
            np.array([0, 0, 0.020])
        ],
        'mimic': {
            'index_pip': {'joint': 'index_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'index_dip': {'joint': 'index_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    },
    'middle': {
        'names': ['middle_mcp_pitch', 'middle_pip', 'middle_dip'],
        'axes': [np.array(x) for x in [[0, 1, 0], [0, 1, 0], [0, 1, 0]]],
        'limits': [(0, 1.3607), (0, 1.8317), (0, 1.628)],
        'offsets': [
            np.array([-0.001397, 0, 0.035623]),
            np.array([-0.0055098, 0, 0.025656]),
            np.array([0, 0, 0.020])
        ],
        'mimic': {
            'middle_pip': {'joint': 'middle_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'middle_dip': {'joint': 'middle_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    },
    'ring': {
        'names': ['ring_mcp_roll', 'ring_mcp_pitch', 'ring_pip', 'ring_dip'],
        'axes': [np.array(x) for x in [[-0.99996, 0, 0.0087265], [0, 1, 0], [0, 1, 0], [0, 1, 0]]],
        'limits': [(0, 0.2181), (0, 1.3607), (0, 1.8317), (0, 0.628)],
        'offsets': [
            np.array([0.0020763, 0, 0.015294]),
            np.array([-0.0013807, 0, 0.035624]),
            np.array([-0.0054686, 0, 0.025665]),
            np.array([0, 0, 0.020])
        ],
        'mimic': {
            'ring_pip': {'joint': 'ring_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'ring_dip': {'joint': 'ring_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    },
    'pinky': {
        'names': ['pinky_mcp_roll', 'pinky_mcp_pitch', 'pinky_pip', 'pinky_dip'],
        'axes': [np.array(x) for x in [[-0.99996, 0, 0.0087265], [0, 1, 0], [0, 1, 0], [0, 1, 0]]],
        'limits': [(0, 0.3489), (0, 1.3607), (0, 1.8317), (0, 0.628)],
        'offsets': [
            np.array([0.0020763, 0, 0.015294]),
            np.array([-0.0013807, 0, 0.035624]),
            np.array([-0.0054686, 0, 0.025665]),
            np.array([0, 0, 0.020])
        ],
        'mimic': {
            'pinky_pip': {'joint': 'pinky_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'pinky_dip': {'joint': 'pinky_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    }
}