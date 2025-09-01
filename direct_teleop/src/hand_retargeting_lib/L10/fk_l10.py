# fk_l10.py
# 专为 LinkerHand L10 模型设计的正向运动学 (Forward Kinematics) 计算模块

import numpy as np

# --- 导入L10专属的依赖 ---
from hand_retargeting_lib.L10.utils_l10 import rotation_matrix, translation_matrix

class HandForwardKinematics_L10:
    def __init__(self, config_module):
        """
        构造函数。
        :param config_module: 导入的 config_l10 模块。
        """
        self.config = config_module

    def calculate_fk_single_finger(self, finger_type, joint_angles):
        """
        计算单个手指链的FK，返回该手指所有连杆末端的局部坐标点列表。
        """
        finger_info = self.config.JOINT_INFO[finger_type]
        base_pos = self.config.FINGER_BASE_POSITIONS[finger_type]

        # 初始变换：从手腕原点平移到手指的基座
        T_current = translation_matrix(base_pos[0], base_pos[1], base_pos[2])
        
        # 存储计算出的关键点位置
        # 注意: L10的结构更复杂，我们返回所有连杆的位置
        keypoints = [T_current[:3, 3]] # 第一个点是手指基座的位置

        # 依次应用每个关节的变换
        for i, angle in enumerate(joint_angles):
            # 获取当前关节的旋转轴
            axis = finger_info['axes'][i]
            # 创建旋转矩阵
            rot_mat = rotation_matrix(axis, angle)
            T_rot = np.eye(4)
            T_rot[:3, :3] = rot_mat
            
            # 应用旋转
            T_current = T_current @ T_rot

            # 应用从当前关节到下一个关节原点的平移
            if i < len(finger_info['joint_origins_xyz']):
                offset = finger_info['joint_origins_xyz'][i]
                T_trans = translation_matrix(offset[0], offset[1], offset[2])
                T_current = T_current @ T_trans
            
            keypoints.append(T_current[:3, 3])

        return keypoints

    def calculate_fk_all_keypoints(self, joint_angles_dict):
        """
        计算所有定义在KEYPOINT_MAP中的关键点，并按顺序返回一个(N,3)的数组。
        """
        num_keypoints = len(self.config.KEYPOINT_MAP)
        results_array = np.zeros((num_keypoints, 3))
        
        # 1. 手腕在原点 (0,0,0)
        results_array[self.config.KEYPOINT_MAP['wrist']] = np.array([0.0, 0.0, 0.0])

        # 2. 掌心使用配置文件中的固定偏移
        results_array[self.config.KEYPOINT_MAP['palm_center']] = self.config.ROBOT_PALM_CENTER_OFFSET

        # 3. 计算每个手指的FK
        for finger in self.config.FINGER_NAMES:
            if finger not in joint_angles_dict:
                continue

            angles_independent = joint_angles_dict[finger]
            
            # 创建一个包含所有关节角度的完整列表，包括模仿关节
            finger_info = self.config.JOINT_INFO[finger]
            num_total_joints = len(finger_info['names'])
            angles_full = [0.0] * num_total_joints
            
            # 填充独立关节的角度
            independent_joint_indices = [i for i, name in enumerate(finger_info['names']) if name not in finger_info.get('mimic', {})]
            for i, angle in zip(independent_joint_indices, angles_independent):
                angles_full[i] = angle
            
            # 计算模仿关节的角度
            if 'mimic' in finger_info:
                for mimic_joint, rule in finger_info['mimic'].items():
                    master_joint_name = rule['joint']
                    master_joint_idx = finger_info['names'].index(master_joint_name)
                    mimic_joint_idx = finger_info['names'].index(mimic_joint)
                    
                    master_angle = angles_full[master_joint_idx]
                    angles_full[mimic_joint_idx] = master_angle * rule['multiplier'] + rule['offset']

            # 调用FK计算这个手指的所有连杆位置
            finger_keypoints_all = self.calculate_fk_single_finger(finger, angles_full)

            # --- 将计算出的连杆位置映射到我们定义的逻辑关键点 ---
            # 这是一个示例映射，您可能需要根据L10的URDF和RViz中的显示进行微调
            # URDF: base -> metacarpal -> proximal -> middle -> distal
            # FK_points: [base_pos, metacarpal_pos, proximal_pos, middle_pos, distal_pos]
            
            # 拇指有5个可动关节，5个连杆
            if finger == 'thumb':
                results_array[self.config.KEYPOINT_MAP['thumb_mcp']] = finger_keypoints_all[3] # thumb_metacarpals 末端
                results_array[self.config.KEYPOINT_MAP['thumb_pip']] = finger_keypoints_all[4] # thumb_proximal 末端
                results_array[self.config.KEYPOINT_MAP['thumb_dip']]  = finger_keypoints_all[5] # thumb_distal 末端
                results_array[self.config.KEYPOINT_MAP['thumb_tip']]  = finger_keypoints_all[5] # 暂时用distal末端代替指尖

            # 其他手指有4个可动关节，4个连杆
            elif finger in ['index', 'ring', 'pinky']:
                results_array[self.config.KEYPOINT_MAP[f'{finger}_mcp']] = finger_keypoints_all[1] # metacarpals 末端
                results_array[self.config.KEYPOINT_MAP[f'{finger}_pip']] = finger_keypoints_all[2] # proximal 末端
                results_array[self.config.KEYPOINT_MAP[f'{finger}_dip']] = finger_keypoints_all[3] # middle 末端
                results_array[self.config.KEYPOINT_MAP[f'{finger}_tip']] = finger_keypoints_all[4] # distal 末端
            
            # 中指只有3个可动关节
            elif finger == 'middle':
                results_array[self.config.KEYPOINT_MAP[f'{finger}_mcp']] = finger_keypoints_all[0] # middle_proximal 的基座，即mcp关节位置
                results_array[self.config.KEYPOINT_MAP[f'{finger}_pip']] = finger_keypoints_all[1] # proximal 末端
                results_array[self.config.KEYPOINT_MAP[f'{finger}_dip']] = finger_keypoints_all[2] # middle 末端
                results_array[self.config.KEYPOINT_MAP[f'{finger}_tip']] = finger_keypoints_all[3] # distal 末端

        return results_array