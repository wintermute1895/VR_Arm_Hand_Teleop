# fk_l10.py
# (为LinkerHand L10模型设计的、运动学正确的正向运动学计算模块)

import numpy as np
import rospy

# 导入L10的专属依赖
from .utils_l10 import rotation_matrix, translation_matrix

class HandForwardKinematics_L10:
    def __init__(self, config_module):
        """
        构造函数。
        :param config_module: 导入的 config_l10 模块。
        """
        self.config = config_module

    def calculate_fk_single_finger(self, finger_type, joint_angles_full):
        """
        [最终正确版] 根据URDF参数计算一个手指上所有连杆末端的位置。
        它使用标准的串联变换矩阵方法。
        """
        finger_info = self.config.JOINT_INFO[finger_type]
        base_pos = self.config.FINGER_BASE_POSITIONS[finger_type]
        
        num_joints_in_chain = len(finger_info['names'])
        if len(joint_angles_full) != num_joints_in_chain:
            raise ValueError(f"FK Error for finger '{finger_type}': Expected {num_joints_in_chain} joint angles, but got {len(joint_angles_full)}.")

        # T_parent 代表父关节坐标系在世界坐标系(手腕)中的位姿
        # 初始时，它是手指的基座
        T_parent = translation_matrix(base_pos[0], base_pos[1], base_pos[2])
        
        # keypoints列表将存储每个连杆末端的位置
        # 第一个点是手指的基座位置，即第一个关节的原点
        keypoints = [T_parent[:3, 3]]

        for i in range(num_joints_in_chain):
            # 1. 获取从父关节到当前关节的平移 (即骨骼/连杆)
            # 注意: 'offsets' 列表的长度应该等于关节数量
            offset = finger_info['offsets'][i]
            T_trans = translation_matrix(offset[0], offset[1], offset[2])

            # 2. 获取当前关节的旋转
            axis = finger_info['axes'][i]
            angle = joint_angles_full[i]
            T_rot = rotation_matrix(axis, angle) # 假设返回4x4齐次矩阵
            
            # 3. 计算当前关节相对于父关节的局部变换
            T_local = T_trans @ T_rot

            # 4. 计算当前关节在世界坐标系中的最终位姿
            T_current_world = T_parent @ T_local
            
            # 5. 当前关节坐标系的原点，就是这个连杆的末端位置
            keypoints.append(T_current_world[:3, 3])
            
            # 6. 对于下一个关节来说，当前的位姿就是它的父位姿
            T_parent = T_current_world
            
        return np.array(keypoints)

    def calculate_fk_all_keypoints(self, joint_angles_dict):
        """
        计算所有手指的所有连杆末端位置。
        返回一个以手指名为键的字典。
        """
        results = {}
        for finger in self.config.ALL_FINGER_NAMES: # 使用ALL_FINGER_NAMES以计算所有模型
            if finger in joint_angles_dict:
                angles = joint_angles_dict[finger]
                num_expected = len(self.config.JOINT_INFO[finger]['names'])
                
                if len(angles) == num_expected:
                    results[finger] = self.calculate_fk_single_finger(finger, angles)
                else:
                    # 如果关节角度数量不匹配，则跳过并打印警告
                    rospy.logwarn_throttle(5.0, f"FK mismatch for finger '{finger}': Expected {num_expected} angles, got {len(angles)}. Skipping.")
        return results

    def calculate_fk_all_fingertips(self, joint_angles_dict):
        """
        一个辅助函数，只计算所有手指的指尖位置。
        """
        results = {}
        for finger in self.config.ALL_FINGER_NAMES:
            if finger in joint_angles_dict:
                all_keypoints = self.calculate_fk_single_finger(finger, joint_angles_dict[finger])
                if all_keypoints.size > 0:
                    results[finger] = all_keypoints[-1] # 指尖是最后一个点
        return results