#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# retargeting.py
# (最终稳定版：实现了纯计算损失函数和安全的日志记录)

import numpy as np
from scipy.optimize import minimize, Bounds
import rospy
import traceback
import copy

# 导入通用的工具函数
from hand_retargeting_lib.L10.utils_l10 import vector_to_angles_dict, angles_dict_to_vector

class HandOptimizer:
    def __init__(self, robot_hand_model, config_module):
        self.model = robot_hand_model
        # 注意：这里我们不再需要 config_module 的静态引用
        # self.config = config_module 
        self.config = config_module # 仍保留，用于访问 LAMBDA_* 等常量
        self.bounds = self._create_bounds()

    def _create_bounds(self):
        """根据传入的config为独立关节创建边界。"""
        b = []
        # 使用 self.model.joint_info 代替 self.config.JOINT_INFO
        joint_info = self.model.joint_info 
        for finger in self.config.FINGER_NAMES:
            finger_info = joint_info.get(finger, {})
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
        self.model.update_human_hand_state(w_dict)
        
        # 运行优化器
        result = minimize(
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

        # 处理并返回结果
        if not result.success:
            # [修正]：这里必须传入 self.model.joint_info
            final_q_dict = vector_to_angles_dict(q_prev_vec, self.model.joint_info)
            return final_q_dict, {"status": "failed", "message": result.message}

        final_q_vec = result.x
        # 传递 self.model.joint_info
        final_q_dict = vector_to_angles_dict(final_q_vec, self.model.joint_info)

        # 优化成功后，安全地获取详细损失信息并记录日志
        final_loss_info = self._loss_function(final_q_vec, q_prev_vec, return_dict=True)
        final_loss_info["status"] = "success"
        
        # 使用节流日志在主控制流中安全地打印调试信息
        rospy.loginfo_throttle(0.1, 
            f"[LOSS] Total: {final_loss_info['total']:.4f} | "
            f"Align: {final_loss_info['align']:.4f}, "
            f"Couple: {final_loss_info['couple']:.4f}, "
            f"Smooth: {final_loss_info['smooth']:.4f}"
        )
        
        return final_q_dict, copy.copy(final_loss_info)

    def _loss_function(self, q_vec, q_prev_vec, return_dict=False):
        try:
            # 1. 关节向量 -> 关节字典
            q_dict = vector_to_angles_dict(q_vec, self.model.joint_info)
            
            # 2. 正向运动学计算
            robot_keypoints = self.model.fk.calculate_fk_all_keypoints(q_dict)
            
            if not robot_keypoints:
                raise ValueError("Forward kinematics returned no keypoints.")
            
            # --- 3. 姿态对齐损失 (loss_align) ---
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
            
            # --- [新增探针和断言] ---
            if np.isnan(loss_align) or np.isinf(loss_align):
                rospy.logerr("[LOSS_DEBUG] CRITICAL: loss_align is NaN or Inf!")
                return 1e10 # 返回一个巨大的值，而不是NaN

            # --- 4. 指间耦合损失 (loss_couple) ---
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
            
            # --- [新增探针和断言] ---
            if np.isnan(loss_couple) or np.isinf(loss_couple):
                rospy.logerr("[LOSS_DEBUG] CRITICAL: loss_couple is NaN or Inf!")
                return 1e10

            # --- 5. 时间平滑损失 (loss_smooth) ---
            loss_smooth = np.sum((q_vec - q_prev_vec)**2)

            # --- [新增探针和断言] ---
            if np.isnan(loss_smooth) or np.isinf(loss_smooth):
                rospy.logerr("[LOSS_DEBUG] CRITICAL: loss_smooth is NaN or Inf!")
                return 1e10

            # --- 6. 计算加权总损失 ---
            weighted_align = self.config.LAMBDA_ALIGNMENT * loss_align
            weighted_couple = self.config.LAMBDA_COORDINATION * loss_couple
            weighted_smooth = self.config.LAMBDA_SMOOTHNESS * loss_smooth
            total_loss = weighted_align + weighted_couple + weighted_smooth
            
            # --- [新增探针] 每隔一段时间打印一次，避免刷屏 ---
            if not hasattr(self, 'loss_call_count'):
                self.loss_call_count = 0
            self.loss_call_count += 1
            if self.loss_call_count % 50 == 0: # 每50次调用打印一次
                rospy.loginfo(f"[LOSS_DEBUG] q_in:{np.round(q_vec, 2)}, L_align:{loss_align:.4f}, L_couple:{loss_couple:.4f}, L_smooth:{loss_smooth:.4f}, Total:{total_loss:.4f}")

            if np.isnan(total_loss): 
                rospy.logerr(f"[LOSS_DEBUG] CRITICAL: total_loss is NaN! Components: align={weighted_align}, couple={weighted_couple}, smooth={weighted_smooth}")
                return 1e10
            
            if return_dict:
                return {           "total": total_loss, 
            "align": weighted_align,
            "couple": weighted_couple, # 确保这里是 couple
            "smooth": weighted_smooth }
            else:
                return total_loss

        except Exception as e:
            rospy.logerr(f"[RETARGETING CRITICAL ERROR] in _loss_function: {e}")
            rospy.logerr(traceback.format_exc())
            return 1e10