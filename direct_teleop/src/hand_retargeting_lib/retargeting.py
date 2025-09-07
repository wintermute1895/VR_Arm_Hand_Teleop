#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# retargeting.py
# (最终稳定版：实现了纯计算损失函数和安全的日志记录)

import numpy as np
from scipy.optimize import minimize, Bounds
import rospy
import traceback

# 导入通用的工具函数
from hand_retargeting_lib.L10.utils_l10 import vector_to_angles_dict, angles_dict_to_vector

class HandOptimizer:
    def __init__(self, robot_hand_model, config_module):
        """
        构造函数。
        :param robot_hand_model: 一个已初始化的 RobotHandModel 对象。
        :param config_module: 对应的配置模块 (e.g., config_l10)。
        """
        self.model = robot_hand_model
        self.config = config_module
        self.bounds = self._create_bounds()

    def _create_bounds(self):
        """根据传入的config为独立关节创建边界。"""
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

    def optimize_q(self, w_dict, q_prev_vec, frame_count):
        """
        主优化函数。它负责调用纯计算的loss函数并处理日志。
        :param w_dict: 以手指为键的人手关键点字典。
        :param q_prev_vec: 上一帧的最终优化结果，用作本帧的初值和平滑目标。
        :param frame_count: 当前帧的帧数，用于节流日志。
        """
        # 1. 更新模型内部的人手状态 (v, delta, omega等)
        self.model.update_human_hand_state(w_dict)
        
        # 2. 运行优化器
        result = minimize(
            # 将一个纯计算的、无副作用的函数传递给优化器
            fun=lambda q_vec: self._loss_function(q_vec, q_prev_vec),
            x0=q_prev_vec,
            method='SLSQP',
            bounds=self.bounds,
            options={
                'maxiter': self.config.MAX_ITER,
                'ftol': self.config.TOLERANCE,
                'eps': 1e-3
            }
        )

        # 3. 处理并返回结果
        if not result.success:
            final_q_dict = vector_to_angles_dict(q_prev_vec, self.config)
            return final_q_dict, {"status": "failed", "message": result.message}

        final_q_vec = result.x
        final_q_dict = vector_to_angles_dict(final_q_vec, self.config)

        # 4. 优化成功后，安全地获取详细损失信息并记录日志
        final_loss_info = self._loss_function(final_q_vec, q_prev_vec, return_dict=True)
        final_loss_info["status"] = "success"
        
        # 使用节流日志在主控制流中安全地打印调试信息
        rospy.loginfo_throttle(0.1, 
            f"[LOSS] Total: {final_loss_info['total']:.4f} | "
            f"Align: {final_loss_info['align']:.4f}, "
            f"Couple: {final_loss_info['couple']:.4f}, "
            f"Smooth: {final_loss_info['smooth']:.4f}"
        )
        
        return final_q_dict, final_loss_info

    def _loss_function(self, q_vec, q_prev_vec, return_dict=False):
        """
        计算总损失。这是一个纯计算函数，没有任何I/O或ROS调用。
        """
        try:
            q_dict = vector_to_angles_dict(q_vec, self.config)
            robot_keypoints = self.model.fk.calculate_fk_all_keypoints(q_dict)
            
            # --- 1. 姿态对齐损失 (loss_align) ---
            loss_align = 0.0
            points_compared = 0
            for finger in self.config.FINGER_NAMES:
                if finger in self.model.v and finger in robot_keypoints:
                    target_pts = self.model.v[finger]
                    robot_pts = robot_keypoints[finger]
                    num_pts = min(len(target_pts), len(robot_pts))
                    if num_pts > 0:
                        error = target_pts[:num_pts] - robot_pts[:num_pts]
                        loss_align += np.sum(error**2)
                        points_compared += num_pts
            if points_compared > 0:
                loss_align /= points_compared
            
            # --- 2. 指间耦合损失 (loss_couple) ---
            loss_couple = 0.0
            robot_tips = {finger: pts[-1] for finger, pts in robot_keypoints.items() if len(pts) > 0}
            if 'thumb' in robot_tips:
                g_thumb_tip = robot_tips['thumb']
                for finger, finger_tip in robot_tips.items():
                    if finger != 'thumb' and finger in self.model.delta:
                        g_i = finger_tip - g_thumb_tip
                        delta_i = self.model.delta[finger]
                        omega_i = self.model.omega.get(finger, 0.0)
                        loss_couple += omega_i * np.sum((delta_i - g_i)**2)

            # --- 3. 时间平滑损失 (loss_smooth) ---
            loss_smooth = np.sum((q_vec - q_prev_vec)**2)

            # --- 4. 计算加权总损失 ---
            weighted_align = self.config.LAMBDA_ALIGNMENT * loss_align
            weighted_couple = self.config.LAMBDA_COORDINATION * loss_couple
            weighted_smooth = self.config.LAMBDA_SMOOTHNESS * loss_smooth
            total_loss = weighted_align + weighted_couple + weighted_smooth
            
            if np.isnan(total_loss): 
                return 1e10
            
            if return_dict:
                return {
                    "total": total_loss, 
                    "align": weighted_align,
                    "couple": weighted_couple, 
                    "smooth": weighted_smooth
                }
            else:
                return total_loss

        except Exception as e:
            # 仅在发生严重错误时打印，这不会频繁发生
            print(f"[RETARGETING CRITICAL ERROR] in _loss_function: {e}")
            traceback.print_exc()
            return 1e10