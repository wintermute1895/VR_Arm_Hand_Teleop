# retargeting.py

import numpy as np
from scipy.optimize import minimize, Bounds
# 确保从正确的utils导入
from utils_1 import vector_to_angles_dict, angles_dict_to_vector


class HandOptimizer:
    # --- [核心修复] 添加完整的__init__构造函数 ---
    def __init__(self, robot_hand_model, config):
        self.model = robot_hand_model
        self.config = config
        self.bounds = self._create_bounds()

        # 初始化q_prev_vec为零位姿态的独立关节向量
        q0_dict = {}
        for finger in config.FINGER_NAMES:
            # 确保使用正确的关节数量
            if finger in config.JOINT_INFO:
                q0_dict[finger] = [0.0] * len(config.JOINT_INFO[finger]['names'])
        self.q_prev_vec = angles_dict_to_vector(q0_dict, self.config)

    # --- 修复结束 ---

    def _create_bounds(self):
        """根据config为独立关节创建边界"""
        b = []
        for finger in self.config.FINGER_NAMES:
            finger_info = self.config.JOINT_INFO.get(finger, {})
            mimic_info = finger_info.get('mimic', {})
            names = finger_info.get('names', [])
            limits = finger_info.get('limits', [])

            for i, joint_name in enumerate(names):
                if joint_name not in mimic_info:
                    if i < len(limits):
                        b.append(limits[i])
        low = [bound[0] for bound in b]
        high = [bound[1] for bound in b]
        return Bounds(low, high)

    def optimize_q(self, w_lhs):
        """主优化函数"""
        self.model.compute_v_target(w_lhs)

        x0 = self.q_prev_vec

        result = minimize(
            fun=self._loss_function,
            x0=x0,
            method='SLSQP',
            bounds=self.bounds,
            options={
                'maxiter': self.config.MAX_ITER,
                'ftol': self.config.TOLERANCE,
                'eps': 1e-4
            }
        )

        final_loss = result.fun if result.success else -1.0

        if not result.success:
            final_q_dict = vector_to_angles_dict(self.q_prev_vec, self.config)
            return final_q_dict, final_loss

        self.q_prev_vec = result.x
        final_q_dict = vector_to_angles_dict(result.x, self.config)

        return final_q_dict, final_loss

    def _loss_function(self, q_vec):
        """
        [最终版 - 引入尺度归一化]
        计算给定关节角度下的总损失。
        """
        q_dict = vector_to_angles_dict(q_vec, self.config)

        fk_keypoints = self.model.fk.calculate_fk_all_keypoints(q_dict)
        v_target = self.model.v

        # 1. 找到一个“尺度基准”
        fk_q0 = self.model._get_initial_fk_keypoints()
        wrist_pos = fk_q0[self.config.KEYPOINT_MAP['wrist']]
        middle_mcp_pos = fk_q0[self.config.KEYPOINT_MAP['middle_mcp']]
        palm_scale = np.linalg.norm(middle_mcp_pos - wrist_pos)

        if palm_scale < 1e-6:
            palm_scale = 1.0

        # 2. 计算归一化后的误差
        error_vec = (fk_keypoints - v_target) / palm_scale
        loss_align = np.sum(error_vec ** 2)

        # --- 平滑损失 (Smoothness Loss) ---
        loss_smooth = np.sum((q_vec - self.q_prev_vec) ** 2)

        # --- 总损失 ---
        total_loss = (self.config.LAMBDA_ALIGNMENT * loss_align +
                      self.config.LAMBDA_SMOOTHNESS * loss_smooth)

        return total_loss