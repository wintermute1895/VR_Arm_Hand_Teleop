# robothandmodel.py

import numpy as np
from direct_teleop.src.hand_retargeting_lib.L21.fk import HandForwardKinematics
from direct_teleop.src.hand_retargeting_lib.L21.utils_1 import translation_matrix, unity_lhs_to_python_rhs
import direct_teleop.src.hand_retargeting_lib.L21.config as config

class RobotHandModel:
    # ... (__init__, _get_initial_fk_keypoints 保持不变) ...
    def __init__(self, config):
        self.config = config
        self.fk = HandForwardKinematics(config)
        self.T_align = np.eye(4)
        self.v = {}
        self.scale_factor = 1.0

    def _get_initial_fk_keypoints(self):
        q_initial = {}
        for finger in config.FINGER_NAMES:
            num_joints = len(config.JOINT_INFO[finger]['names'])
            q_initial[finger] = [0.0] * num_joints
        return self.fk.calculate_fk_all_keypoints(q_initial)

    def _create_frame(self, keypoints_rhs, label=""):
        """
        [最终版 - PICO优先]
        根据RHS关键点，构建一个稳健的右手坐标系。
        现在直接使用传入的palm_center点。
        """
        # [# AI-AUDIT & REFACTOR] 直接从数组中获取所有需要的点
        wrist_pos = keypoints_rhs[config.KEYPOINT_MAP['wrist']]
        palm_center = keypoints_rhs[config.KEYPOINT_MAP['palm_center']]
        middle_mcp = keypoints_rhs[config.KEYPOINT_MAP['middle_mcp']]
        index_mcp = keypoints_rhs[config.KEYPOINT_MAP['index_mcp']]
        pinky_mcp = keypoints_rhs[config.KEYPOINT_MAP['little_mcp']]  # 在config中是little

        # Y-axis (向前): 从手腕指向掌心，这是一个非常稳定的方向
        y_axis = palm_center - wrist_pos
        y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-8)

        # 临时X轴 (宽度方向): 从小指根部指向食指根部
        temp_x_axis = index_mcp - pinky_mcp

        # Z-axis (向上): 手掌的法线方向
        z_axis = np.cross(temp_x_axis, y_axis)
        z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-8)

        # 最终X轴 (向右): 确保三轴严格正交
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-8)

        print(f"\n--- {label} Frame Calculation (RHS) ---")
        print(f"Y-axis (Forward): {y_axis}")
        print(f"Z-axis (Up)     : {z_axis}")
        print(f"X-axis (Right)  : {x_axis}")

        R = np.stack([x_axis, y_axis, z_axis], axis=1)
        return R

    # ... (_calculate_scale_factor, calibrate, compute_v_target 保持不变) ...
    def _calculate_scale_factor(self, human_hand_rhs, robot_hand_rhs):
        """根据手掌大小计算人手到机器人手的尺度因子"""
        # 我们使用一个稳定的测量标准：手腕到中指根部的距离
        human_wrist = human_hand_rhs[config.KEYPOINT_MAP['wrist']]
        human_middle_mcp = human_hand_rhs[config.KEYPOINT_MAP['middle_mcp']]
        human_size = np.linalg.norm(human_middle_mcp - human_wrist)

        robot_wrist = robot_hand_rhs[config.KEYPOINT_MAP['wrist']]
        robot_middle_mcp = robot_hand_rhs[config.KEYPOINT_MAP['middle_mcp']]
        robot_size = np.linalg.norm(robot_middle_mcp - robot_wrist)

        if human_size < 1e-6:
            return 1.0

        scale = robot_size / human_size
        print(
            f"  [Scale Calibration] Human Size: {human_size:.2f}, Robot Size: {robot_size:.2f}, Scale Factor: {scale:.3f}")
        return scale

    def calibrate(self, w_star_lhs):
        """
        [最终版]
        使用第一帧LHS数据进行校准，现在包含尺度校准。
        """
        # 1. 净化和获取机器人骨架 (保持不变)
        w_star_rhs = unity_lhs_to_python_rhs(w_star_lhs)
        fk_q0_dict = self._get_initial_fk_keypoints()

        # 2. [# AI-AUDIT & REFACTOR] 计算并存储尺度因子
        self.scale_factor = self._calculate_scale_factor(w_star_rhs, fk_q0_dict)

        # 3. 对齐旋转 (保持不变)
        frame_human = self._create_frame(w_star_rhs, "Human Hand")
        frame_robot = np.eye(3)
        R_align = frame_robot @ frame_human.T

        # 4. 构建最终的齐次变换矩阵 (保持不变)
        T_rot = np.eye(4)
        T_rot[:3, :3] = R_align
        w_wrist_rhs = w_star_rhs[config.KEYPOINT_MAP['wrist']]
        T_trans_to_origin = translation_matrix(-w_wrist_rhs[0], -w_wrist_rhs[1], -w_wrist_rhs[2])
        self.T_align = T_rot @ T_trans_to_origin
        print("Calibration complete.")

    def compute_v_target(self, w_lhs):
        """
        [最终版]
        计算目标v，现在应用尺度因子。
        """
        # 1. 净化和对齐人手数据 (保持不变)
        w_rhs = unity_lhs_to_python_rhs(w_lhs)
        w_homo = np.hstack([w_rhs, np.ones((w_rhs.shape[0], 1))])
        w_aligned_homo = (self.T_align @ w_homo.T).T
        w_aligned = w_aligned_homo[:, :3]

        # [# AI-AUDIT & REFACTOR] 在重建v之前，应用尺度因子
        # 我们将对齐后的人手关键点相对于其手腕进行缩放
        w_aligned_wrist = w_aligned[config.KEYPOINT_MAP['wrist']]
        w_scaled = w_aligned_wrist + (w_aligned - w_aligned_wrist) * self.scale_factor

        # 2. 后续逻辑使用缩放后的w_scaled，而不是w_aligned
        fk_q0_dict = self._get_initial_fk_keypoints()
        v_final = fk_q0_dict.copy()

        for finger in config.FINGER_NAMES:
            # ... (此循环内的其余代码保持不变, 但确保它使用的是 w_scaled)
            # 例如: human_segment_direction = w_scaled[p_end_idx] - w_scaled[p_start_idx]
            mcp_idx = config.KEYPOINT_MAP[f'{finger}_mcp']
            pip_idx = config.KEYPOINT_MAP[f'{finger}_pip']
            dip_idx = config.KEYPOINT_MAP[f'{finger}_dip']
            tip_idx = config.KEYPOINT_MAP[f'{finger}_tip']

            joint_chain_indices = [mcp_idx, pip_idx, dip_idx, tip_idx]

            for i in range(len(joint_chain_indices) - 1):
                p_start_idx = joint_chain_indices[i]
                p_end_idx = joint_chain_indices[i + 1]

                v_start_point = v_final[p_start_idx]

                # 使用缩放后的人手数据来提取方向
                human_segment_direction = w_scaled[p_end_idx] - w_scaled[p_start_idx]
                norm = np.linalg.norm(human_segment_direction)
                if norm > 1e-8:
                    human_segment_direction /= norm

                robot_segment_length = np.linalg.norm(fk_q0_dict[p_end_idx] - fk_q0_dict[p_start_idx])

                v_final[p_end_idx] = v_start_point + human_segment_direction * robot_segment_length

        v_final[config.KEYPOINT_MAP['wrist']] = fk_q0_dict[config.KEYPOINT_MAP['wrist']]
        v_final[config.KEYPOINT_MAP['palm_center']] = fk_q0_dict[config.KEYPOINT_MAP['palm_center']]

        self.v = v_final
        return self.v