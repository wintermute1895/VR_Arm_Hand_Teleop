# robothandmodel_l10.py
# 专为 LinkerHand L10 模型设计的机器人手模型类

import numpy as np

# --- 导入L10专属的依赖 ---
from hand_retargeting_lib.L10.fk_l10 import HandForwardKinematics_L10
from hand_retargeting_lib.L10.utils_l10 import translation_matrix, unity_lhs_to_python_rhs

class RobotHandModel_L10:
    def __init__(self, config_module):
        """
        构造函数。
        :param config_module: 导入的 config_l10 模块。
        """
        self.config = config_module
        self.fk = HandForwardKinematics_L10(self.config)
        self.T_align = np.eye(4)
        self.v = {}
        self.scale_factor = 1.0

    def _get_initial_fk_keypoints(self):
        """获取机器手在零位姿态下的所有关键点位置。"""
        q_initial = {}
        for finger in self.config.FINGER_NAMES:
            num_joints = len(self.config.JOINT_INFO[finger]['names'])
            q_initial[finger] = [0.0] * num_joints
        return self.fk.calculate_fk_all_keypoints(q_initial)

    def _create_frame(self, keypoints_rhs, label=""):
        """
        根据给定的关键点，构建一个稳健的右手坐标系。
        """
        # 从数组中获取所有需要的点
        wrist_pos = keypoints_rhs[self.config.KEYPOINT_MAP['wrist']]
        palm_center = keypoints_rhs[self.config.KEYPOINT_MAP['palm_center']]
        middle_mcp = keypoints_rhs[self.config.KEYPOINT_MAP['middle_mcp']]
        index_mcp = keypoints_rhs[self.config.KEYPOINT_MAP['index_mcp']]
        # !! 关键修改: L10的config中使用的是'pinky'而不是'little' !!
        pinky_mcp = keypoints_rhs[self.config.KEYPOINT_MAP['pinky_mcp']] 

        # Y-axis (向前): 从手腕指向掌心
        y_axis = palm_center - wrist_pos
        y_axis /= (np.linalg.norm(y_axis) + 1e-8)

        # 临时X轴 (宽度方向): 从小指根部指向食指根部
        temp_x_axis = index_mcp - pinky_mcp

        # Z-axis (向上): 手掌的法线方向
        z_axis = np.cross(temp_x_axis, y_axis)
        z_axis /= (np.linalg.norm(z_axis) + 1e-8)

        # 最终X轴 (向右): 确保三轴严格正交
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= (np.linalg.norm(x_axis) + 1e-8)

        # (调试打印可以暂时保留或注释掉)
        # print(f"\n--- {label} Frame Calculation (RHS) ---")
        # print(f"Y-axis (Forward): {y_axis}")
        # print(f"Z-axis (Up)     : {z_axis}")
        # print(f"X-axis (Right)  : {x_axis}")

        R = np.stack([x_axis, y_axis, z_axis], axis=1)
        return R

    def _calculate_scale_factor(self, human_hand_rhs, robot_hand_rhs):
        """根据手掌大小计算人手到机器人手的尺度因子。"""
        human_wrist = human_hand_rhs[self.config.KEYPOINT_MAP['wrist']]
        human_middle_mcp = human_hand_rhs[self.config.KEYPOINT_MAP['middle_mcp']]
        human_size = np.linalg.norm(human_middle_mcp - human_wrist)

        robot_wrist = robot_hand_rhs[self.config.KEYPOINT_MAP['wrist']]
        robot_middle_mcp = robot_hand_rhs[self.config.KEYPOINT_MAP['middle_mcp']]
        robot_size = np.linalg.norm(robot_middle_mcp - robot_wrist)

        if human_size < 1e-6:
            return 1.0

        scale = robot_size / human_size
        print(f"  [Scale Calibration] Human Size: {human_size:.3f}m, Robot Size: {robot_size:.3f}m, Scale Factor: {scale:.3f}")
        return scale

    def calibrate(self, w_star_lhs):
        """
        使用第一帧数据进行校准（旋转对齐+尺度计算）。
        """
        # 注意: 如果PICO数据已经是RHS，这一步可能需要调整
        w_star_rhs = unity_lhs_to_python_rhs(w_star_lhs)
        fk_q0_dict = self._get_initial_fk_keypoints()

        self.scale_factor = self._calculate_scale_factor(w_star_rhs, fk_q0_dict)

        frame_human = self._create_frame(w_star_rhs, "Human Hand")
        frame_robot = np.eye(3) # 假设机器人手腕坐标系是标准坐标系
        R_align = frame_robot @ frame_human.T

        T_rot = np.eye(4)
        T_rot[:3, :3] = R_align
        w_wrist_rhs = w_star_rhs[self.config.KEYPOINT_MAP['wrist']]
        T_trans_to_origin = translation_matrix(-w_wrist_rhs[0], -w_wrist_rhs[1], -w_wrist_rhs[2])
        self.T_align = T_rot @ T_trans_to_origin
        print("Calibration complete.")

    def compute_v_target(self, w_lhs):
        """
        计算优化器需要的目标关键点'v'。
        """
        w_rhs = unity_lhs_to_python_rhs(w_lhs)
        w_homo = np.hstack([w_rhs, np.ones((w_rhs.shape[0], 1))])
        w_aligned_homo = (self.T_align @ w_homo.T).T
        w_aligned = w_aligned_homo[:, :3]

        w_aligned_wrist = w_aligned[self.config.KEYPOINT_MAP['wrist']]
        w_scaled = w_aligned_wrist + (w_aligned - w_aligned_wrist) * self.scale_factor

        fk_q0_dict = self._get_initial_fk_keypoints()
        v_final = fk_q0_dict.copy()

        for finger in self.config.FINGER_NAMES:
            # 根据config中的KEYPOINT_MAP获取索引
            mcp_idx = self.config.KEYPOINT_MAP[f'{finger}_mcp']
            pip_idx = self.config.KEYPOINT_MAP[f'{finger}_pip']
            dip_idx = self.config.KEYPOINT_MAP[f'{finger}_dip']
            tip_idx = self.config.KEYPOINT_MAP[f'{finger}_tip']
            
            joint_chain_indices = [mcp_idx, pip_idx, dip_idx, tip_idx]

            for i in range(len(joint_chain_indices) - 1):
                p_start_idx = joint_chain_indices[i]
                p_end_idx = joint_chain_indices[i + 1]

                v_start_point = v_final[p_start_idx]

                human_segment_direction = w_scaled[p_end_idx] - w_scaled[p_start_idx]
                norm = np.linalg.norm(human_segment_direction)
                if norm > 1e-8:
                    human_segment_direction /= norm

                robot_segment_length = np.linalg.norm(fk_q0_dict[p_end_idx] - fk_q0_dict[p_start_idx])
                v_final[p_end_idx] = v_start_point + human_segment_direction * robot_segment_length

        v_final[self.config.KEYPOINT_MAP['wrist']] = fk_q0_dict[self.config.KEYPOINT_MAP['wrist']]
        v_final[self.config.KEYPOINT_MAP['palm_center']] = fk_q0_dict[self.config.KEYPOINT_MAP['palm_center']]

        self.v = v_final
        return self.v