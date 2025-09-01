# utils.py

import numpy as np
from scipy.spatial.transform import Rotation


# --- AI-AUDIT & REFACTOR ---
# 融合了工程师版本中更健壮的scipy实现和耦合关节处理逻辑。

def sigmoid(x, k=10, c=0.5):
    """Sigmoid 函数"""
    return 1 / (1 + np.exp(-k * (x - c)))


def rotation_matrix(axis, angle):
    """根据轴和角度（弧度）创建4x4齐次旋转矩阵"""
    axis = np.asarray(axis)
    # 避免除以零
    norm = np.linalg.norm(axis)
    if norm < 1e-8:
        return np.eye(3)
    rot_vec = axis / norm * angle
    rotation = Rotation.from_rotvec(rot_vec)

    return rotation.as_matrix()


def translation_matrix(dx, dy, dz):
    """创建4x4齐次平移矩阵"""
    return np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])


def unity_lhs_to_python_rhs(keypoints):
    """将(N, 3)的关键点从Unity的LHS转换为Python的RHS"""
    keypoints_rhs = keypoints.copy()
    keypoints_rhs[:, 2] *= -1
    return keypoints_rhs


def angles_dict_to_vector(q_dict, config):
    """将完整的关节角度字典，根据耦合关系，转换为独立驱动向量。"""
    vec = []
    for finger in config.FINGER_NAMES:
        finger_info = config.JOINT_INFO[finger]
        mimic_info = finger_info.get('mimic', {})
        for i, joint_name in enumerate(finger_info['names']):
            # 只将非模仿（独立）关节添加到向量中
            if joint_name not in mimic_info:
                vec.append(q_dict[finger][i])
    return np.array(vec)


def vector_to_angles_dict(q_vec, config):
    """将独立驱动向量，根据耦合关系，扩展为完整的关节角度字典，并应用限制。"""
    q_dict = {}
    vec_idx = 0
    for finger in config.FINGER_NAMES:
        finger_info = config.JOINT_INFO[finger]
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


def convert_q_to_robot_command_l21(q_dict, config):
    """
    [最终版 - L21专用]
    将关节角度q_dict转换为L21灵巧手API所需的25元素pose列表 (0-255)。
    映射关系源自Linker Hand L21官方开发者文档。
    """
    # Key: pose[] 列表的索引, Value: (我们config中的手指名, 我们config中的关节索引)
    mapping = {
        0: ('thumb', 2), 1: ('index', 1), 2: ('middle', 1), 3: ('ring', 1), 4: ('little', 1),
        5: ('thumb', 0), 6: ('index', 0), 7: ('middle', 0), 8: ('ring', 0), 9: ('little', 0),
        10: ('thumb', 1),
        11: None, 12: None, 13: None, 14: None,
        15: ('thumb', 3),  # "大拇指中部" -> 暂时联动到拇指尖部
        16: None, 17: None, 18: None, 19: None,
        20: ('thumb', 3), 21: ('index', 2), 22: ('middle', 2), 23: ('ring', 2), 24: ('little', 2)
    }

    pose = [128] * 25  # L21的pose列表长度是25

    for pose_idx, joint_ref in mapping.items():
        if joint_ref is None:
            pose[pose_idx] = 0  # 预留位设为0
            continue

        finger_name, joint_idx = joint_ref

        if finger_name in config.JOINT_INFO and joint_idx < len(config.JOINT_INFO[finger_name]['names']):
            angle_rad = q_dict[finger_name][joint_idx]
            min_limit, max_limit = config.JOINT_INFO[finger_name]['limits'][joint_idx]

            if abs(max_limit - min_limit) < 1e-6:
                normalized_value = 0.5
            else:
                normalized_value = (angle_rad - min_limit) / (max_limit - min_limit)

            pose_value = int(np.clip((1.0 - normalized_value) * 255, 0, 255))
            pose[pose_idx] = pose_value

    return pose