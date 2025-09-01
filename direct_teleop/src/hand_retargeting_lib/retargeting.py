# retargeting.py
# (为LinkerHand L10模型优化，并包含完整的L_couple成本项)

import numpy as np
from scipy.optimize import minimize, Bounds

# --- 关键：严格按照您的项目结构进行导入 ---
from hand_retargeting_lib.L10.utils_l10 import vector_to_angles_dict, angles_dict_to_vector, sigmoid
import hand_retargeting_lib.L10.config_l10 as config

class HandOptimizer:
    def __init__(self, robot_hand_model):
        """
        构造函数。
        :param robot_hand_model: 一个已初始化的 RobotHandModel_L10 对象。
        """
        self.model = robot_hand_model
        # 注意: 即使外部传入了config, 内部逻辑也强依赖导入的L10 config
        self.config = config 
        self.bounds = self._create_bounds()
        self.q_prev_vec = self._create_initial_vector()

    def _create_bounds(self):
        """根据L10的config为独立关节创建边界。"""
        b = []
        for finger in self.config.FINGER_NAMES:
            finger_info = self.config.JOINT_INFO.get(finger, {})
            mimic_info = finger_info.get('mimic', {})
            names = finger_info.get('names', [])
            limits = finger_info.get('limits', [])
            for i, joint_name in enumerate(names):
                if joint_name not in mimic_info and i < len(limits):
                    b.append(limits[i])
        low = [bound[0] for bound in b]
        high = [bound[1] for bound in b]
        return Bounds(low, high)

    def _create_initial_vector(self):
        """根据L10的config创建零位姿的独立关节向量。"""
        q0_dict = {}
        for finger in self.config.FINGER_NAMES:
            num_independent_joints = 0
            finger_info = self.config.JOINT_INFO.get(finger, {})
            mimic_info = finger_info.get('mimic', {})
            for name in finger_info.get('names', []):
                if name not in mimic_info:
                    num_independent_joints += 1
            q0_dict[finger] = [0.0] * num_independent_joints
        return angles_dict_to_vector(q0_dict, self.config)

    def optimize_q(self, w_lhs):
        """主优化函数。"""
        self.model.compute_v_target(w_lhs)
        result = minimize(
            fun=self._loss_function,
            x0=self.q_prev_vec,
            method='SLSQP',
            bounds=self.bounds,
            options={
                'maxiter': self.config.MAX_ITER,
                'ftol': self.config.TOLERANCE,
                'eps': 1e-4
            }
        )

        if not result.success:
            final_q_dict = vector_to_angles_dict(self.q_prev_vec, self.config)
            return final_q_dict, -1.0

        self.q_prev_vec = result.x
        final_q_dict = vector_to_angles_dict(result.x, self.config)
        return final_q_dict, result.fun

    def _loss_function(self, q_vec):
        """
        计算给定关节角度下的总损失，包含姿态对齐、指间耦合和平滑三项。
        """
        # --- 1. 数据准备 ---
        q_dict = vector_to_angles_dict(q_vec, self.config)
        fk_keypoints = self.model.fk.calculate_fk_all_keypoints(q_dict)
        v_target = self.model.v

        # --- 2. 姿态对齐成本 (L_align) ---
        fk_q0 = self.model._get_initial_fk_keypoints()
        wrist_pos = fk_q0[self.config.KEYPOINT_MAP['wrist']]
        middle_mcp_pos = fk_q0[self.config.KEYPOINT_MAP['middle_mcp']]
        palm_scale = np.linalg.norm(middle_mcp_pos - wrist_pos)
        if palm_scale < 1e-6: palm_scale = 1.0
        
        error_vec = (fk_keypoints - v_target) / palm_scale
        loss_align = np.sum(error_vec ** 2)

        # --- 3. 指间耦合成本 (L_couple) ---
        loss_couple = 0.0
        thumb_tip_idx = self.config.KEYPOINT_MAP['thumb_tip']
        human_thumb_tip_pos = v_target[thumb_tip_idx]
        robot_thumb_tip_pos = fk_keypoints[thumb_tip_idx]
        
        # L10的config中pinky替代了little
        coupling_fingers = ['index', 'middle', 'ring', 'pinky']
        for finger in coupling_fingers:
            finger_tip_idx = self.config.KEYPOINT_MAP[f'{finger}_tip']
            human_finger_tip_pos = v_target[finger_tip_idx]
            robot_finger_tip_pos = fk_keypoints[finger_tip_idx]
            
            human_relative_vec = human_finger_tip_pos - human_thumb_tip_pos
            robot_relative_vec = robot_finger_tip_pos - robot_thumb_tip_pos
            
            rho_min = self.config.COUPLING_PARAMS['rho_min']
            rho_max = self.config.COUPLING_PARAMS['rho_max']
            sigma = self.config.COUPLING_PARAMS['sigma']
            tau = self.config.COUPLING_PARAMS['tau']
            
            human_dist = np.linalg.norm(human_relative_vec)
            rho_t = 1.0 - np.clip((human_dist - rho_min) / (rho_max - rho_min), 0, 1)
            beta_t = sigmoid(rho_t, k=sigma, c=tau)
            
            finger_couple_loss = beta_t * np.sum((human_relative_vec - robot_relative_vec)**2)
            loss_couple += finger_couple_loss

        # --- 4. 时间平滑成本 (L_smooth) ---
        loss_smooth = np.sum((q_vec - self.q_prev_vec)**2)

        # --- 5. 计算总损失 ---
        total_loss = (self.config.LAMBDA_ALIGNMENT * loss_align +
                      self.config.LAMBDA_COUPLE * loss_couple +
                      self.config.LAMBDA_SMOOTHNESS * loss_smooth)
                      
        return total_loss