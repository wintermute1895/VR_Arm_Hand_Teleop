# fk.py

import numpy as np
from direct_teleop.src.hand_retargeting_lib.L21.utils_1 import rotation_matrix, translation_matrix
import direct_teleop.src.hand_retargeting_lib.L21.config as config

class HandForwardKinematics:
    def __init__(self, config):
        self.config = config

    def calculate_fk(self, finger_type, joint_angles):
        """计算单个手指链的FK，返回局部坐标点列表"""
        finger_info = config.JOINT_INFO[finger_type]
        base_pos = config.FINGER_BASE_POSITIONS[finger_type]

        T_world = translation_matrix(base_pos[0], base_pos[1], base_pos[2])
        keypoints = [T_world[:3, 3]]  # MCP joint position

        for i, angle in enumerate(joint_angles):
            axis = finger_info['axes'][i]
            rot_mat = rotation_matrix(axis, angle)
            T_rot = np.eye(4)
            T_rot[:3, :3] = rot_mat

            T_world = T_world @ T_rot

            if i < len(finger_info['joint_origins_xyz']):
                offset = finger_info['joint_origins_xyz'][i]
                T_trans = translation_matrix(offset[0], offset[1], offset[2])
                T_world = T_world @ T_trans

            keypoints.append(T_world[:3, 3])

        return keypoints

    def calculate_fk_all_keypoints(self, joint_angles_dict):
        """计算所有关键点，并按KEYPOINT_MAP的顺序返回一个(N,3)的数组"""
        num_keypoints = len(config.KEYPOINT_MAP)
        results_array = np.zeros((num_keypoints, 3))

        # 1. 手腕在原点
        results_array[config.KEYPOINT_MAP['wrist']] = np.array([0.0, 0.0, 0.0])

        # 2. 掌心使用固定偏移
        results_array[config.KEYPOINT_MAP['palm_center']] = config.ROBOT_PALM_CENTER_OFFSET

        # 3. 计算每个手指的FK
        for finger in config.FINGER_NAMES:
            num_joints = len(config.JOINT_INFO[finger]['names'])
            angles = joint_angles_dict.get(finger, [0.0] * num_joints)

            # 处理mimic关节
            if 'mimic' in config.JOINT_INFO[finger]:
                for mimic_joint, rule in config.JOINT_INFO[finger]['mimic'].items():
                    source_joint_name = rule['joint']
                    source_finger = \
                    [f for f, info in config.JOINT_INFO.items() if source_joint_name in info['names']][0]
                    source_joint_idx = config.JOINT_INFO[source_finger]['names'].index(source_joint_name)
                    mimic_joint_idx = config.JOINT_INFO[finger]['names'].index(mimic_joint)

                    angles[mimic_joint_idx] = angles[source_joint_idx] * rule['multiplier'] + rule['offset']

            finger_keypoints = self.calculate_fk(finger, angles)

            # 填充到结果数组中
            mcp_idx = config.KEYPOINT_MAP[f'{finger}_mcp']
            pip_idx = config.KEYPOINT_MAP[f'{finger}_pip']
            dip_idx = config.KEYPOINT_MAP[f'{finger}_dip']
            tip_idx = config.KEYPOINT_MAP[f'{finger}_tip']

            results_array[mcp_idx] = finger_keypoints[0]
            results_array[pip_idx] = finger_keypoints[1]
            results_array[dip_idx] = finger_keypoints[2]
            results_array[tip_idx] = finger_keypoints[3]

        return results_array