# config_l10.py
# 为 LinkerHand L10 左手模型生成的配置文件
# 所有位置单位已从URDF中的米(m)转换为标准单位米(m)

import numpy as np

# --- 权重系数 (保持不变) ---
HAND_MODEL = "L10"
LAMBDA_ALIGNMENT = 5.0
LAMBDA_SMOOTHNESS = 0.1
LAMBDA_COUPLE = 10.0

# 优化器参数 (保持不变)
MAX_ITER = 10
TOLERANCE = 1e-3

# ==== 权威关键点定义 (适配L10的连杆结构) ====
# 注意: 我们将URDF中的连杆映射到通用的MCP, PIP, DIP, TIP概念
# wrist -> hand_base_link (逻辑原点)
# mcp   -> ..._metacarpals 或 ..._proximal 的关节原点
# pip   -> ..._proximal 或 ..._middle 的关节原点
# dip   -> ..._middle 或 ..._distal 的关节原点
# tip   -> ..._distal 的末端 (需要手动测量或在fk中定义)
KEYPOINT_MAP = {
    'wrist': 0,
    'thumb_mcp': 1, 'thumb_pip': 2, 'thumb_dip': 3, 'thumb_tip': 4,
    'index_mcp': 5, 'index_pip': 6, 'index_dip': 7, 'index_tip': 8,
    'middle_mcp': 9, 'middle_pip': 10, 'middle_dip': 11, 'middle_tip': 12,
    'ring_mcp': 13, 'ring_pip': 14, 'ring_dip': 15, 'ring_tip': 16,
    'pinky_mcp': 17, 'pinky_pip': 18, 'pinky_dip': 19, 'pinky_tip': 20,
    'palm_center': 21 # 这是一个逻辑点，需要手动估计
}

# ==== 机器人参数 (从L10 URDF中提取并转换为米) ====
# 手掌中心相对于手腕(hand_base_link)的偏移量，这是一个估算值，您可以在RViz中微调
ROBOT_PALM_CENTER_OFFSET = np.array([-0.003, 0.0, 0.080])

# 每个手指链的基座相对于手腕(hand_base_link)的位置
FINGER_BASE_POSITIONS = {
    'thumb':  np.array([-0.013419, -0.012551, 0.060602]), # thumb_cmc_roll origin
    'index':  np.array([-0.0021643, -0.026654, 0.13253]),  # index_mcp_roll origin
    'middle': np.array([-0.0021316, -0.0076542, 0.15281]), # middle_mcp_pitch origin
    'ring':   np.array([-0.0021643, 0.011346, 0.13253]),   # ring_mcp_roll origin
    'pinky':  np.array([-0.00012074, 0.030346, 0.12755]),  # pinky_mcp_roll origin
}

# 详细的关节信息
JOINT_INFO = {
    'thumb': {
        'names': ['thumb_cmc_roll', 'thumb_cmc_yaw', 'thumb_cmc_pitch', 'thumb_mcp', 'thumb_ip'],
        'axes': [np.array([-0.99996, 0, -0.0087265]), np.array([-0.008517, -0.21782, 0.97595]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(0.0, 1.1339), (0.0, 1.9189), (0.0, 0.5149), (0.0, 0.7152), (0.0, 0.7763)],
        'joint_origins_xyz': [np.array([0.035797, 0.00065879, 0.00045944]), np.array([0.0046051, -0.014383, -0.0051478]), np.array([0.0061722, 0.0, 0.047968]), np.array([-0.00017064, 0.0, 0.038665])],
        'mimic': {
            'thumb_mcp': {'joint': 'thumb_cmc_pitch', 'multiplier': 1.3898, 'offset': 0.0},
            'thumb_ip': {'joint': 'thumb_cmc_pitch', 'multiplier': 1.508, 'offset': 0.0}
        }
    },
    'index': {
        'names': ['index_mcp_roll', 'index_mcp_pitch', 'index_pip', 'index_dip'],
        'axes': [np.array([0.99996, 0, 0.0087265]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(0.0, 0.2181), (0.0, 1.3607), (0.0, 1.8317), (0.0, 0.628)],
        'joint_origins_xyz': [np.array([0.0020763, 0.0, 0.015294]), np.array([-0.0013807, 0.0, 0.035624]), np.array([-0.0054686, 0.0, 0.025665])],
        'mimic': {
            'index_pip': {'joint': 'index_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'index_dip': {'joint': 'index_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    },
    'middle': {
        'names': ['middle_mcp_pitch', 'middle_pip', 'middle_dip'], # 注意：这个手指在URDF中没有mcp_roll
        'axes': [np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(0.0, 1.3607), (0.0, 1.8317), (0.0, 1.628)],
        'joint_origins_xyz': [np.array([-0.001397, 0.0, 0.035623]), np.array([-0.0055098, 0.0, 0.025656])],
        'mimic': {
            'middle_pip': {'joint': 'middle_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'middle_dip': {'joint': 'middle_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    },
    'ring': {
        'names': ['ring_mcp_roll', 'ring_mcp_pitch', 'ring_pip', 'ring_dip'],
        'axes': [np.array([-0.99996, 0, 0.0087265]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(0.0, 0.2181), (0.0, 1.3607), (0.0, 1.8317), (0.0, 0.628)],
        'joint_origins_xyz': [np.array([0.0020763, 0.0, 0.015294]), np.array([-0.0013807, 0.0, 0.035624]), np.array([-0.0054686, 0.0, 0.025665])],
        'mimic': {
            'ring_pip': {'joint': 'ring_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'ring_dip': {'joint': 'ring_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    },
    'pinky': { # 在URDF中，小指被称为'pinky'
        'names': ['pinky_mcp_roll', 'pinky_mcp_pitch', 'pinky_pip', 'pinky_dip'],
        'axes': [np.array([-0.99996, 0, 0.0087265]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])],
        'limits': [(0.0, 0.3489), (0.0, 1.3607), (0.0, 1.8317), (0.0, 0.628)],
        'joint_origins_xyz': [np.array([0.0020763, 0.0, 0.015294]), np.array([-0.0013807, 0.0, 0.035624]), np.array([-0.0054686, 0.0, 0.025665])],
        'mimic': {
            'pinky_pip': {'joint': 'pinky_mcp_pitch', 'multiplier': 1.3462, 'offset': 0.0},
            'pinky_dip': {'joint': 'pinky_mcp_pitch', 'multiplier': 0.4616, 'offset': 0.0}
        }
    },
}

# 确保FINGER_NAMES的键与JOINT_INFO的键完全一致
FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']
ALL_FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']

COUPLING_PARAMS = {
    'rho_min': 0.01,  # 1cm, 指尖距离小于这个值时，耦合权重开始增加
    'rho_max': 0.10,  # 10cm, 指尖距离大于这个值时，耦合权重为0
    'sigma': 10,      # Sigmoid 函数的陡峭程度
    'tau': 0.5        # Sigmoid 函数的激活阈值
}
