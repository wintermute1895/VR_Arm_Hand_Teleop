# main.py

import cv2
import numpy as np
import mediapipe as mp
from config import Config
from retargeting import HandOptimizer
from robothandmodel import RobotHandModel

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

def extract_keypoints(hand_landmarks, image_shape):
    h, w, _ = image_shape
    keypoints_dict = {}
    finger_indices = {
        'thumb': [1, 2, 3, 4], 'index': [5, 6, 7, 8], 'middle': [9, 10, 11, 12],
        'ring': [13, 14, 15, 16], 'little': [17, 18, 19, 20]
    }
    landmarks = np.array([[lm.x * w, lm.y * h, lm.z * w] for lm in hand_landmarks.landmark])
    keypoints_dict['wrist'] = np.expand_dims(landmarks[0], axis=0)
    for finger, indices in finger_indices.items():
        keypoints_dict[finger] = landmarks[indices]
    return keypoints_dict

# --- 核心改动：恢复此函数的完整输出 ---
def save_debug_data_to_file(model, file_handle, loss_value):
    """保存详细的调试参数到 hand_data.txt"""
    np.set_printoptions(precision=3, suppress=True)
    def write_line(text=""): file_handle.write(text + '\n')

    write_line("\n" + "="*25 + f" FRAME DATA " + "="*25)
    
    # 1. 关节角度 (q_t)
    write_line("\n--- 1. Robot Joint Angles (q_t) [Radians] ---")
    for finger, angles in model.q.items():
        write_line(f"  {finger.capitalize():<7}: {np.array(angles)}")
    
    # 2. 原始人手关键点 (w)
    write_line("\n--- 2. Raw Human Keypoints (w) ---")
    for finger, points in model.w.items():
        write_line(f"  {finger.capitalize():<7}: \n{points}")

    # 3. 调整后的人手目标点 (v)
    write_line("\n--- 3. Adjusted Target Keypoints (v) ---")
    for finger, points in model.v.items():
        write_line(f"  {finger.capitalize():<7}: \n{points}")

    # 4. 机器人正向运动学结果 (FK)
    write_line("\n--- 4. Robot Forward Kinematics (FK) ---")
    fk_points = model.fk.calculate_fk_all_keypoints(model.q)
    for finger, points in fk_points.items():
        write_line(f"  {finger.capitalize():<7}: \n{points}")
        
    # 9. 最终损失函数值
    write_line("\n--- 9. Total Loss Function Value ---")
    write_line(f"  Loss: {loss_value:.4f}")
    write_line("\n" + "="*62 + "\n")
    file_handle.flush()
# --- 改动结束 ---

def normalize_and_save_move_data(q_dict, config, file_handle):
    """
    将16个独立关节的角度，按照特定顺序和规则归一化到 0-255 并保存。
    """
    joint_order = [
        ('thumb', 2, 'mcp_flex'),  ('index', 1, 'mcp_flex'), ('middle', 1, 'mcp_flex'), ('ring', 1, 'mcp_flex'), ('little', 1, 'mcp_flex'),
        ('thumb', 0, 'abd'), ('index', 0, 'abd'),  ('middle', 0, 'abd'),  ('ring', 0, 'abd'),  ('little', 0, 'abd'),
        ('thumb', 1, 'roll'),
        ('thumb', 3, 'pip_flex'), ('index', 2, 'pip_flex'), ('middle', 2, 'pip_flex'), ('ring', 2, 'pip_flex'), ('little', 2, 'pip_flex')
    ]

    angle_data = []
    for finger_name, joint_idx, joint_type in joint_order:
        finger_info = config.JOINT_INFO[finger_name]
        min_limit, max_limit = finger_info['limits'][joint_idx]
        angle = q_dict[finger_name][joint_idx]
        
        if abs(max_limit - min_limit) < 1e-6:
            normalized_value = 0
        else:
            normalized_value = (max_limit - angle) / (max_limit - min_limit)
        
        uint8_value = int(np.clip(normalized_value * 255, 0, 255))
        
        if finger_name != 'thumb' and (joint_type == 'mcp_flex' or joint_type == 'abd'):
            uint8_value = int(np.clip(uint8_value, 30, 210))
            
        angle_data.append(uint8_value)

    output_string = '\t'.join(map(str, angle_data))
    file_handle.write(output_string + '\n')
    file_handle.flush()


def main():
    config = Config()
    q0 = {}
    for finger in config.FINGER_NAMES:
        num_joints = len(config.JOINT_INFO[finger]['names'])
        q0[finger] = [0.0] * num_joints
    w0 = {finger: np.zeros((4, 3)) for finger in config.FINGER_NAMES}
    
    robot_model = RobotHandModel(config, q0, w0)
    optimizer = HandOptimizer(robot_model, config)
    
    is_calibrated = False
    frame_count = 0
    last_loss = 0.0

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    print("请将手掌完全张开，正对摄像头进行校准...")

    try:
        with open('hand_data.txt', 'w') as debug_file, \
             open('move_data.txt', 'w') as move_file:
            
            while cap.isOpened():
                success, image_original = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue

                image_rgb = cv2.cvtColor(image_original, cv2.COLOR_BGR2RGB)
                results = hands.process(image_rgb)

                if results.multi_hand_landmarks:
                    hand_landmarks = results.multi_hand_landmarks[0]
                    mp_drawing.draw_landmarks(image_original, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    human_keypoints = extract_keypoints(hand_landmarks, image_original.shape)

                    if not is_calibrated:
                        robot_model.calibrate(human_keypoints)
                        print("校准完成！开始运动重定位。")
                        is_calibrated = True
                    else:
                        frame_count += 1
                        q_t, final_loss = optimizer.optimize_q(human_keypoints)
                        last_loss = final_loss
                        robot_model.update_robot_hand_state(q_t)
                        
                        if frame_count % 5 == 0:
                            print(f"Frame {frame_count}: Saving data (Loss: {last_loss:.4f})...")
                            save_debug_data_to_file(robot_model, debug_file, last_loss)
                            normalize_and_save_move_data(robot_model.q, config, move_file)
                
                cv2.imshow('Hand Retargeting', image_original)
                if cv2.waitKey(5) & 0xFF == ord('q'):
                    break
    finally:
        print("Closing resources.")
        cap.release()
        cv2.destroyAllWindows()
        hands.close()

if __name__ == '__main__':
    main()