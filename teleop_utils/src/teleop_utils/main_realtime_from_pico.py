# main_realtime_with_hand.py (V5 - 最终修正版)

import socket
import json
import numpy as np
import time
import sys

# --- 1. 导入灵巧手API ---
try:
    from LinkerHand.linker_hand_api import LinkerHandApi
except ImportError:
    print("\n[严重错误] 未找到 'LinkerHandApi' 库。")
    print("请确保您已在正确的环境中，并已安装SDK依赖。")
    sys.exit(1)

# 导入我们所有的核心模块
from config import Config
from pico_data_handler import PICODataHandler
from robothandmodel import RobotHandModel
from retargeting import HandOptimizer
from utils_1 import convert_q_to_robot_command_l21


def main():
    hand = None
    # --- 2. 初始化灵巧手硬件 ---
    try:
        print("正在初始化灵巧手硬件 (L21)...")
        hand = LinkerHandApi(hand_type="right", hand_joint="L21")
        print("✅ 灵巧手API初始化成功。")

        if hasattr(hand, 'clear_faults'):
            print("--> 正在复位灵巧手，清除历史故障...")
            hand.clear_faults()
            time.sleep(0.5)

        realtime_speed = [180] * 25
        hand.set_speed(speed=realtime_speed)
        print(f"✅ 灵巧手速度已设置为: {realtime_speed}")

    except Exception as e:
        print(f"\n[严重错误] 灵巧手硬件初始化失败: {e}")
        print("请检查硬件连接、电源和驱动是否正常。程序将退出。")
        sys.exit(1)

    # --- 初始化其他模块 ---
    print("\n正在初始化核心模块...")
    config = Config()
    pico_handler = PICODataHandler()
    robot_model = RobotHandModel(config)
    optimizer = HandOptimizer(robot_model, config)
    print("✅ 核心模块初始化完成。")

    # --- 设置UDP接收器 ---
    UDP_IP = "0.0.0.0"
    UDP_PORT = 9999
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)
    print(f"✅ UDP监听器已启动在 {UDP_IP}:{UDP_PORT} (等待来自PICO的数据)")

    # --- 初始化状态变量 ---
    is_calibrated = False
    frame_count = 0
    last_print_time = time.time()
    last_command_time = time.time()
    COMMAND_INTERVAL = 0.02

    try:
        print("\n等待来自PICO的数据流...")
        while True:
            try:
                data, addr = sock.recvfrom(8192)
                json_string = data.decode('utf-8').rstrip('\x00')
                pico_packet = json.loads(json_string)

                # --- [核心修正] 直接传递原始的'joints'数据 ---
                # 不再进行任何中间转换，因为pico_data_handler期望的就是原始的字典列表
                if isinstance(pico_packet, dict) and 'joints' in pico_packet:
                    raw_pico_joints = pico_packet['joints']
                else:
                    print("[警告] 收到的数据格式不符合预期，跳过。")
                    continue
                # --- 修正结束 ---

                w_lhs = pico_handler.process_pico_data(raw_pico_joints)

            except socket.timeout:
                print("等待数据超时...", end='\r')
                continue
            except (json.JSONDecodeError, KeyError, IndexError, TypeError) as e:
                print(f"\n数据解析或处理错误: {e}, 跳过此帧。")
                continue

            # --- 校准与重定向逻辑 (保持不变) ---
            if not is_calibrated:
                print("\n接收到第一帧有效数据，正在进行校准...")
                robot_model.calibrate(w_lhs)
                is_calibrated = True
                print("\n✅ 校准完成，开始实时重定向。")
            else:
                q_t, final_loss = optimizer.optimize_q(w_lhs)
                pose_to_send = convert_q_to_robot_command_l21(q_t, config)

                # --- 发送指令到物理灵巧手 (保持不变) ---
                current_time = time.time()
                if current_time - last_command_time > COMMAND_INTERVAL:
                    hand.finger_move(pose=pose_to_send)
                    last_command_time = current_time

                # 优化打印逻辑
                frame_count += 1
                if frame_count % 30 == 0 or (time.time() - last_print_time) > 1.0:
                    print(f"Frame: {frame_count} | Loss: {final_loss:.4f} | Scale: {robot_model.scale_factor:.3f}")
                    last_print_time = time.time()

    except KeyboardInterrupt:
        print("\n程序被用户中断。")
    finally:
        if hand:
            print("\n正在将灵巧手恢复到安全的中立姿态...")
            try:
                neutral_pose = [128] * 25
                hand.finger_move(pose=neutral_pose)
                time.sleep(1)
                print("✅ 已发送恢复姿态指令。")
            except Exception as e:
                print(f"[错误] 在恢复姿态时出错: {e}")

        sock.close()
        print("UDP监听器已关闭。程序结束。")

if __name__ == "__main__":
    main()