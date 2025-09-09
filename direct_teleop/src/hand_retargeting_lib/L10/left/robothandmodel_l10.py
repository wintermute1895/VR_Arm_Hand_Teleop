# robothandmodel_l10.py
# (最终修正版：统一了内部函数名，严格遵循原始代码逻辑)

import numpy as np
from scipy.spatial.transform import Rotation

from ..fk_l10 import HandForwardKinematics_L10
from ..utils_l10 import sigmoid, translation_matrix

class RobotHandModel_L10:
    def __init__(self, config_module, q0_dict, w0_dict):
        self.config = config_module
        self.fk = HandForwardKinematics_L10(self.config)
        self.q = q0_dict
        self.w = w0_dict
        
        self.T_align = np.eye(4)
        self.d_max, self.d_min = {}, {}
        self.v, self.delta, self.d_contact, self.omega = {}, {}, {}, {}

    def update_human_hand_state(self, new_w_dict):
        self.w = new_w_dict
        self._compute_v()
        self._compute_delta()
        self._compute_d_contact()
        self._compute_omega()

    def update_robot_hand_state(self, new_q_dict):
        self.q = new_q_dict
        
    # --- !! 核心修正 1：将函数名统一为 _get_base_pose_keypoints !! ---
    def _get_base_pose_keypoints(self):
        """
        计算机器人手在“完全张开”(max_limit)姿态下的关键点。
        这个姿态是所有后续计算的基准。
        """
        q_open = {finger: [limit[1] for limit in self.config.JOINT_INFO[finger]['limits']] 
                  for finger in self.config.ALL_FINGER_NAMES}
        return self.fk.calculate_fk_all_keypoints(q_open)

    def _create_frame(self, p_wrist, p_index_mcp, p_pinky_mcp, label=""):
        # (这个函数保持您之前的、健壮的版本)
        try:
            x_axis = p_index_mcp - p_pinky_mcp
            temp_vec = ((p_index_mcp + p_pinky_mcp) / 2) - p_wrist
            if np.linalg.norm(x_axis) < 1e-6 or np.linalg.norm(temp_vec) < 1e-6:
                raise ValueError("Input points are co-linear or too close.")
            x_axis /= np.linalg.norm(x_axis)
            z_axis = np.cross(temp_vec, x_axis)
            if np.linalg.norm(z_axis) < 1e-6:
                raise ValueError("Input points are co-linear, cannot form a frame.")
            z_axis /= np.linalg.norm(z_axis)
            y_axis = np.cross(z_axis, x_axis)
            rotation_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
            determinant = np.linalg.det(rotation_matrix)
            if abs(determinant - 1.0) > 1e-3:
                raise ValueError(f"Invalid right-handed frame created (determinant is {determinant:.4f}).")
            return rotation_matrix
        except Exception as e:
            print(f"Error in _create_frame for '{label}': {e}")
            return None

    def calibrate(self, w_star_dict):
        print("Starting calibration based on original 3-point frame logic...")
        # !! 核心修正 2：确保调用统一后的函数名 !!
        fk_q0 = self._get_base_pose_keypoints()
        
        required_keys = ['wrist', 'index', 'pinky']
        if not all(key in w_star_dict and len(w_star_dict[key]) > 0 for key in required_keys):
            raise ValueError("Required keypoints (wrist, index, pinky) not found.")
        
        w_wrist = w_star_dict['wrist'][0]
        w_index_mcp = w_star_dict['index'][0]
        w_pinky_mcp = w_star_dict['pinky'][0]
        
        robot_wrist_origin = np.array([0.0, 0.0, 0.0])
        robot_index_mcp = fk_q0['index'][0]
        robot_pinky_mcp = fk_q0['pinky'][0]
        
        frame_human = self._create_frame(w_wrist, w_index_mcp, w_pinky_mcp, "Human Hand")
        frame_robot = self._create_frame(robot_wrist_origin, robot_index_mcp, robot_pinky_mcp, "Robot Hand")
        
        if frame_human is None or frame_robot is None:
            raise RuntimeError("Failed to create valid coordinate frames during calibration.")

        R_align = frame_robot @ frame_human.T
        T_rot = np.eye(4); T_rot[:3, :3] = R_align
        T_trans_to_origin = translation_matrix(-w_wrist[0], -w_wrist[1], -w_wrist[2])
        self.T_align = T_rot @ T_trans_to_origin
        
        new_d_max, new_d_min = {}, {}
        if 'thumb' in w_star_dict and len(w_star_dict['thumb']) > 0:
            thumb_tip_star = w_star_dict['thumb'][-1]
            for finger, points in w_star_dict.items():
                if finger == 'thumb' or not isinstance(points, np.ndarray) or points.size == 0: continue
                distance = np.linalg.norm(points[-1] - thumb_tip_star)
                new_d_max[finger] = distance
                new_d_min[finger] = 0.005
        
        self.d_max, self.d_min = new_d_max, new_d_min
        print("Calibration finished successfully!")

    def _compute_v(self):
        w_aligned = {}
        for key, w_points in self.w.items():
            if not isinstance(w_points, np.ndarray) or w_points.size == 0: continue
            w_points_homo = np.hstack([w_points, np.ones((w_points.shape[0], 1))])
            w_aligned_homo = (self.T_align @ w_points_homo.T).T
            w_aligned[key] = w_aligned_homo[:, :3]

        v_final = {}
        # !! 核心修正 3：确保调用统一后的函数名 !!
        fk_q0 = self._get_base_pose_keypoints()

        for finger in self.config.FINGER_NAMES:
            if finger not in w_aligned or finger not in fk_q0: continue
            
            w_points_aligned = w_aligned[finger]
            num_points = min(len(w_points_aligned), len(fk_q0[finger]))
            if num_points == 0: continue

            v_points_list = [None] * num_points
            v_points_list[0] = fk_q0[finger][0]
            for j in range(1, num_points):
                segment_vec_aligned = w_points_aligned[j] - w_points_aligned[j-1]
                norm_segment = np.linalg.norm(segment_vec_aligned)
                direction = segment_vec_aligned / norm_segment if norm_segment > 1e-6 else np.array([0,0,0])
                
                robot_segment_len = np.linalg.norm(fk_q0[finger][j] - fk_q0[finger][j-1])
                final_segment_vec = direction * robot_segment_len
                v_points_list[j] = v_points_list[j-1] + final_segment_vec
                
            v_final[finger] = np.array(v_points_list)
        self.v = v_final
        
    def _compute_delta(self):
        if 'thumb' not in self.v or not isinstance(self.v['thumb'], np.ndarray) or self.v['thumb'].size == 0:
            self.delta = {}; return
        self.delta = {}
        thumb_tip = self.v['thumb'][-1]
        for finger, points in self.v.items():
            if finger == 'thumb' or not isinstance(points, np.ndarray) or points.size == 0: continue
            finger_tip = points[-1]
            self.delta[finger] = finger_tip - thumb_tip
    
    def _compute_d_contact(self):
        self.d_contact = {}
        for finger, delta_vec in self.delta.items():
            if finger not in self.d_min or finger not in self.d_max: continue
            current_distance = np.linalg.norm(delta_vec)
            d_min_val, d_max_val = self.d_min[finger], self.d_max[finger]
            if abs(d_max_val - d_min_val) < 1e-6: normalized = 0.0
            else: normalized = (current_distance - d_min_val) / (d_max_val - d_min_val)
            self.d_contact[finger] = np.clip(1.0 - normalized, 0.0, 1.0)

    def _compute_omega(self):
        self.omega = {}
        for finger in self.delta.keys():
            if finger not in self.d_contact: continue
            self.omega[finger] = sigmoid(self.d_contact[finger], k=self.config.SIGMOID_K, c=self.config.SIGMOID_C)