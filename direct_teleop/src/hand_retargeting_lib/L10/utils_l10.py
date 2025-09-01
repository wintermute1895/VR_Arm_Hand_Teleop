# utils_l10.py
# 包含了与L10灵巧手模型适配的特定工具函数

import numpy as np
from scipy.spatial.transform import Rotation
import rospy
# 导入L10的专属配置
from hand_retargeting_lib.L10 import config_l10 as config

# ==============================================================================
# 通用工具函数 (与L21版本完全相同)
# ==============================================================================

def sigmoid(x, k=10, c=0.5):
    """Sigmoid 函数"""
    return 1 / (1 + np.exp(-k * (x - c)))

def rotation_matrix(axis, angle):
    """根据轴和角度（弧度）创建3x3旋转矩阵"""
    axis = np.asarray(axis)
    norm = np.linalg.norm(axis)
    if norm < 1e-8:
        return np.eye(3)
    rot_vec = axis / norm * angle
    rotation = Rotation.from_rotvec(rot_vec)
    
    if hasattr(rotation, 'as_matrix'):
        return rotation.as_matrix()
    else:
        return rotation.as_dcm() # 兼容旧版scipy

def translation_matrix(dx, dy, dz):
    """创建4x4齐次平移矩阵"""
    return np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])

def unity_lhs_to_python_rhs(keypoints):
    """将(N, 3)的关键点从Unity的LHS转换为Python的RHS"""
    # 这个函数现在可能不需要了，因为PICO数据已经是RHS
    # 但保留它以备不时之需
    keypoints_rhs = keypoints.copy()
    keypoints_rhs[:, 2] *= -1
    return keypoints_rhs

def angles_dict_to_vector(q_dict, config_module):
    """将完整的关节角度字典，根据耦合关系，转换为独立驱动向量。"""
    vec = []
    for finger in config_module.FINGER_NAMES:
        finger_info = config_module.JOINT_INFO[finger]
        mimic_info = finger_info.get('mimic', {})
        for i, joint_name in enumerate(finger_info['names']):
            if joint_name not in mimic_info:
                vec.append(q_dict[finger][i])
    return np.array(vec)

def vector_to_angles_dict(q_vec, config_module):
    """将独立驱动向量，根据耦合关系，扩展为完整的关节角度字典，并应用限制。"""
    q_dict = {}
    vec_idx = 0
    for finger in config_module.FINGER_NAMES:
        finger_info = config_module.JOINT_INFO[finger]
        num_joints = len(finger_info['names'])
        angles = [0.0] * num_joints
        independent_joints_map = {}

        for i, joint_name in enumerate(finger_info['names']):
            if joint_name not in finger_info.get('mimic', {}):
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
# L10 专用硬件指令转换函数 (核心修改)
# ==============================================================================

def convert_q_to_l10_command(q_dict):
    sdk_joint_mapping = [
        ('thumb',  'thumb_mcp'), ('thumb',  'thumb_cmc_yaw'), ('index',  'index_mcp_pitch'),
        ('middle', 'middle_mcp_pitch'), ('ring',   'ring_mcp_pitch'), ('pinky',  'pinky_mcp_pitch'),
        ('index',  'index_mcp_roll'), ('ring',   'ring_mcp_roll'), ('pinky',  'pinky_mcp_roll'),
        ('thumb',  'thumb_cmc_roll'),
    ]
    position_cmd = [0] * 10
    
    # 确保传入的q_dict是完整的（包含模仿关节）
    full_q_dict = q_dict

    for i, (finger, joint_name) in enumerate(sdk_joint_mapping):
        try:
            joint_idx = config.JOINT_INFO[finger]['names'].index(joint_name)
            angle_rad = full_q_dict[finger][joint_idx]
            min_limit, max_limit = config.JOINT_INFO[finger]['limits'][joint_idx]

            if abs(max_limit - min_limit) < 1e-6:
                normalized_value = 0.0
            else:
                normalized_value = (angle_rad - min_limit) / (max_limit - min_limit)
            
            pose_value = int(np.clip((1.0 - normalized_value) * 255, 0, 255))
            position_cmd[i] = pose_value
        except (KeyError, ValueError, IndexError) as e:

            rospy.logwarn_throttle(5.0, f"Could not process joint '{finger}/{joint_name}' for L10 command. Using default. Error: {e}")
            default_values = [255, 128, 255, 255, 255, 255, 128, 128, 128, 255]
            position_cmd[i] = default_values[i]

    return position_cmd