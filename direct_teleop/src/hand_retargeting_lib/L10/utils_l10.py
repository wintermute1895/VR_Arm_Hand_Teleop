# utils_l10.py
# (严格遵循原始代码逻辑，并加入了L10专属的硬件适配)

import numpy as np
from scipy.spatial.transform import Rotation
import rospy

# 导入L10的专属配置
from .left import config_l10 as config

# ==============================================================================
# 通用工具函数 (逻辑与算法设计者原始代码一致)
# ==============================================================================

def sigmoid(x, k=10, c=0.5):
    """Sigmoid 函数，用于计算 omega 权重"""
    return 1 / (1 + np.exp(-k * (x - c)))

def rotation_matrix(axis, angle):
    """根据轴和角度创建 4x4 旋转矩阵"""
    axis = np.asarray(axis)
    # 增加对零向量的保护
    norm = np.linalg.norm(axis)
    if norm < 1e-8:
        return np.eye(4)
        
    rot_vec = axis / norm * angle
    rotation = Rotation.from_rotvec(rot_vec)
    rot_mat = np.eye(4)
    # 兼容新旧scipy版本
    rot_mat[:3, :3] = rotation.as_matrix() if hasattr(rotation, 'as_matrix') else rotation.as_dcm()
    return rot_mat

def translation_matrix(dx, dy, dz):
    """创建 4x4 平移矩阵"""
    return np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])

def angles_dict_to_vector(q_dict, config_module):
    """
    将完整的关节角度字典，根据L10的耦合关系，转换为独立驱动向量。
    (逻辑与原始代码一致)
    """
    vec = []
    # 使用config.FINGER_NAMES来确保只处理活动的手指
    for finger in config.FINGER_NAMES:
        finger_info = config.JOINT_INFO[finger]
        mimic_info = finger_info.get('mimic', {})
        angles_for_finger = q_dict.get(finger, [])
        for i, joint_name in enumerate(finger_info['names']):
            if joint_name not in mimic_info:
                if i < len(angles_for_finger):
                    vec.append(angles_for_finger[i])
                else:
                    vec.append(0.0) # 如果字典中缺少，则用0填充
    return np.array(vec)

def vector_to_angles_dict(q_vec, config_module):
    """
    将独立驱动向量，根据L10的耦合关系，扩展为完整的关节角度字典。
    (逻辑与原始代码一致)
    """
    q_dict = {}
    vec_idx = 0
    for finger in config.FINGER_NAMES:
        finger_info = config.JOINT_INFO[finger]
        num_joints = len(finger_info['names'])
        angles = [0.0] * num_joints
        
        independent_joints_map = {}
        for i, joint_name in enumerate(finger_info['names']):
            if joint_name not in finger_info.get('mimic', {}):
                if vec_idx < len(q_vec):
                    raw_angle = q_vec[vec_idx]
                    joint_limits = finger_info['limits'][i]
                    angles[i] = np.clip(raw_angle, joint_limits[0], joint_limits[1])
                    independent_joints_map[joint_name] = angles[i]
                    vec_idx += 1
        
        mimic_info = finger_info.get('mimic', {})
        for joint_name, mimic_data in mimic_info.items():
            mimic_idx = finger_info['names'].index(joint_name)
            master_joint_name = mimic_data['joint']
            
            if master_joint_name in independent_joints_map:
                master_angle = independent_joints_map[master_joint_name]
                calculated_angle = master_angle * mimic_data['multiplier'] + mimic_data['offset']
                mimic_limits = finger_info['limits'][mimic_idx]
                angles[mimic_idx] = np.clip(calculated_angle, mimic_limits[0], mimic_limits[1])

        q_dict[finger] = angles
        
    return q_dict

# ==============================================================================
# L10 专用硬件指令转换函数 (您的适配层)
# ==============================================================================

def convert_q_to_l10_command(q_dict,config_module):
    """
    将关节角度字典 q_dict 转换为L10硬件API所需的10元素position列表 (0-255)。
    映射关系和顺序严格遵循L10 SDK文档。
    """
    # SDK文档定义的10个驱动关节顺序
    sdk_joint_mapping = [
        ('thumb',  'thumb_mcp'),        # joint1: 拇指 根部弯曲
        ('thumb',  'thumb_cmc_yaw'),    # joint2: 拇指 侧摆
        ('index',  'index_mcp_pitch'),  # joint3: 食指 根部弯曲
        ('middle', 'middle_mcp_pitch'), # joint4: 中指 根部弯曲
        ('ring',   'ring_mcp_pitch'),   # joint5: 无名指 根部弯曲
        ('pinky',  'pinky_mcp_pitch'),  # joint6: 小指 根部弯曲
        ('index',  'index_mcp_roll'),   # joint7: 食指 侧摆
        ('ring',   'ring_mcp_roll'),    # joint8: 无名指 侧摆
        ('pinky',  'pinky_mcp_roll'),   # joint9: 小指 侧摆
        ('thumb',  'thumb_cmc_roll'),   # joint10: 拇指 旋转
    ]

    position_cmd = [0] * 10
    full_q_dict = q_dict

    for i, (finger, joint_name) in enumerate(sdk_joint_mapping):
        try:
            joint_idx = config_module.JOINT_INFO[finger]['names'].index(joint_name)
            angle_rad = full_q_dict[finger][joint_idx]
            min_limit, max_limit = config_module.JOINT_INFO[finger]['limits'][joint_idx]

            if abs(max_limit - min_limit) < 1e-6:
                normalized_value = 0.0
            else:
                normalized_value = (angle_rad - min_limit) / (max_limit - min_limit)
            
            # 使用线性反向映射，0弧度(min_limit) -> 255, max_limit弧度 -> 0
            pose_value = int(np.clip((1.0 - normalized_value) * 255, 0, 255))
            position_cmd[i] = pose_value
        except (KeyError, ValueError, IndexError) as e:
            rospy.logwarn_throttle(5.0, f"Could not process joint '{finger}/{joint_name}' for L10 command. Using default. Error: {e}")
            default_values = [255, 128, 255, 255, 255, 255, 128, 128, 128, 255]
            position_cmd[i] = default_values[i]

    return position_cmd