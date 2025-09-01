# realtime_control.py (最终修正版 - 已根据精确映射表重写)

import cv2
import numpy as np
import mediapipe as mp
import time
import argparse
import sys
import os

# =============================================================================
# 动态路径设置
# =============================================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../../.."))
sys.path.append(target_dir)
from LinkerHand.linker_hand_api import LinkerHandApi
# =============================================================================

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

# =================================================================
# 核心修正：重写此函数以严格匹配用户提供的20个位置映射
# =================================================================
def convert_to_robot_pose(q_dict, config):
    """
    将关节角度q_dict严格按照API定义的20个位置转换为0-255的指令列表。
    """
    # 定义20个位置分别对应的 (手指名称, 关节索引)
    # 这是根据您的映射表和config.py文件共同推断出来的
    # 'pinky' 是 'little' 的别名
    mapping = {
        0: ('thumb', 2),    # 拇指根部弯曲
        1: ('index', 1),    # 食指根部弯曲
        2: ('middle', 1),   # 中指根部弯曲
        3: ('ring', 1),     # 无名指根部弯曲
        4: ('little', 1),   # 小指根部弯曲
        5: ('thumb', 0),    # 拇指侧摆
        6: ('index', 0),    # 食指侧摆
        7: ('middle', 0),   # 中指侧摆
        8: ('ring', 0),     # 无名指侧摆
        9: ('little', 0),   # 小指侧摆
        10: ('thumb', 1),   # 拇指横摆 (旋转)
        11: None,           # 预留
        12: None,           # 预留
        13: None,           # 预留
        14: None,           # 预留
        15: ('thumb', 3),   # 拇指尖部弯曲
        16: ('index', 2),   # 食指末端弯曲
        17: ('middle', 2),  # 中指末端弯曲
        18: ('ring', 2),    # 无名指末端弯曲
        19: ('little', 2)   # 小指末端弯曲
    }

    final_pose = [0] * 20 # 创建一个长度为20的空列表

    for i in range(20):
        joint_info = mapping.get(i)
        
        # 如果是预留位
        if joint_info is None:
            final_pose[i] = 0
            continue
            
        finger_name, joint_idx = joint_info

        # 如果是禁用的拇指
        if finger_name == 'thumb':
            final_pose[i] = 200
            continue

        # 对于活动的手指，计算并填充值
        angle = q_dict[finger_name][joint_idx]
        min_limit, max_limit = config.JOINT_INFO[finger_name]['limits'][joint_idx]

        if abs(max_limit - min_limit) < 1e-6:
            normalized_value = 0.5
        else:
            # 新的、正确的公式
            normalized_value = (max_limit - angle) / (max_limit - min_limit)

        uint8_value = int(np.clip(normalized_value * 255, 0, 255))
        final_pose[i] = uint8_value
        
    return final_pose
# =================================================================

def main(args):
    config = Config()
    q0 = {}
    for finger in config.ALL_FINGER_NAMES:
        num_joints = len(config.JOINT_INFO[finger]['names'])
        q0[finger] = [0.0] * num_joints
    w0 = {finger: np.zeros((4, 3)) for finger in config.ALL_FINGER_NAMES}
    
    robot_model = RobotHandModel(config, q0, w0)
    optimizer = HandOptimizer(robot_model, config)
    
    try:
        print("[INFO] 正在为“左手”初始化机械手API...")
        hand = LinkerHandApi(hand_joint=args.hand_joint, hand_type='left', can="PCAN_USBBUS1")

        print("[INFO] 正在使能机械手电机...")
        hand.set_enable()
        time.sleep(0.1)
        print("[成功] 机械手电机已使能。")

        speed = [250] * 5
        hand.set_speed(speed=speed)
        time.sleep(0.5)
        print(f"[成功] 机械手API已初始化，速度设置为 {speed[0]}。")
    except Exception as e:
        print(f"[错误] 初始化机械手失败: {e}")
        print("[提示] 将以模拟模式运行，机械手不会移动。")
        hand = None

    is_calibrated = False
    frame_count = 0
    
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    print("\n[提示] 摄像头已开启。请将您的“左手”放入画面中。")
    print("[提示] 请先将手掌完全张开，正对摄像头进行初始校准。")
    
    try:
        while cap.isOpened():
            success, image_original = cap.read()
            if not success: continue

            image_rgb = cv2.cvtColor(image_original, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)
            image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                mp_drawing.draw_landmarks(image_bgr, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                human_keypoints = extract_keypoints(hand_landmarks, image_bgr.shape)

                if not is_calibrated:
                    try:
                        robot_model.calibrate(human_keypoints)
                        print("[成功] 校准完成！已开始实时重定位 (拇指已禁用)。")
                        is_calibrated = True
                    except ValueError as e:
                        print(f"[校准错误] {e}。请保持手部平稳并完全可见。")
                        continue
                
                if is_calibrated:
                    frame_count += 1
                    q_t, final_loss = optimizer.optimize_q(human_keypoints)
                    robot_model.update_robot_hand_state(q_t)
                    
                    if frame_count % 10 == 0:
                        pose_to_send = convert_to_robot_pose(robot_model.q, config)
                        if hand:
                            hand.finger_move(pose=pose_to_send)
                        print(f"帧 {frame_count}: 损失: {final_loss:.4f} | 发送姿态: {pose_to_send}")

            cv2.imshow('实时手部重定位 (左手模式)', image_bgr)
            if cv2.waitKey(5) & 0xFF == ord('q'): break
    finally:
        print("\n[提示] 正在关闭程序...")
        cap.release()
        cv2.destroyAllWindows()
        hands.close()
        if hand:
            print("[提示] 机械手正在回到初始姿态。")
            initial_pose = [128] * 20
            # 将预留位设为0
            for i in [11, 12, 13, 14]:
                initial_pose[i] = 0
            hand.finger_move(pose=initial_pose)
            time.sleep(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="实时手部-机械手控制脚本")
    parser.add_argument("--hand_joint", type=str, default="L20", help="机械手关节类型 (必须是 L20)")
    args = parser.parse_args()
    main(args)