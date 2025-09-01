# mapping_explorer.py (基于官方文档的最终版)

import time
import sys

# --- 依赖配置: 尝试导入真实的API，如果失败则使用模拟API ---
try:
    from LinkerHand.linker_hand_api import LinkerHandApi

    IS_MOCK_MODE = False
except ImportError:
    print("=" * 60)
    print("警告: 未找到 'LinkerHandApi'。将以【模拟模式】运行。")
    print("=" * 60)


    class MockLinkerHandApi:
        def __init__(self, hand_type, hand_joint):
            self.hand_type = hand_type
            self.hand_joint = hand_joint
            print(f"--- MOCK API: Initialized for {self.hand_type} {self.hand_joint} ---")

        def set_speed(self, speed):
            print(f"--- MOCK API: Speed set to {speed} ---")

        def finger_move(self, pose):
            print(f"--- MOCK API: Moving to pose (len={len(pose)}): {pose} ---")

        # 为模拟API添加 clear_faults 方法，以保证程序结构一致性
        def clear_faults(self):
            print("--- MOCK API: clear_faults() called. ---")


    LinkerHandApi = MockLinkerHandApi
    IS_MOCK_MODE = True

print (IS_MOCK_MODE)

def  main():
    # 在 try 块外部预先定义 hand 变量
    hand = None
    try:
        # --- 1. 初始化与设置 ---
        print("正在初始化 LinkerHandApi (L21) for right hand...")
        hand = LinkerHandApi(hand_type="right", hand_joint="L21")
        print("✅ API 初始化成功。")

        # 【关键步骤】根据官方文档，调用 clear_faults() 来复位硬件
        # hasattr() 会安全地检查方法是否存在
        if hasattr(hand, 'clear_faults'):
            print("--> 正在清除历史故障码，复位灵巧手...")
            hand.clear_faults()
            time.sleep(0.5)  # 等待复位指令完成
        else:
            print("[警告] 在您的SDK版本中未找到 clear_faults 方法。")

        print("正在设置安全低速...")
        low_speed = [150] * 25
        hand.set_speed(speed=low_speed)
        print(f"✅ 速度已设置为: {low_speed}")
        time.sleep(1)

        # --- 2. 定义姿态与映射表 ---
        pose_length = 25
        neutral_pose = [128] * pose_length
        inferred_mapping = {
            0: "拇指根部弯曲", 1: "食指根部弯曲", 2: "中指根部弯曲", 3: "无名指根部弯曲", 4: "小指根部弯曲",
            5: "拇指侧摆", 6: "食指侧摆", 7: "中指侧摆", 8: "无名指侧摆", 9: "小指侧摆",
            10: "拇指横滚/旋转", 11: "预留", 12: "预留", 13: "预留", 14: "预留",
            15: "大拇指中部弯曲", 16: "预留", 17: "预留", 18: "预留", 19: "预留",
            20: "大拇指指尖弯曲", 21: "食指指尖弯曲", 22: "中指指尖弯曲", 23: "无名指指尖弯曲", 24: "小指指尖弯曲"
        }

        # --- 3. 执行测试循环 ---
        print("\n" + "=" * 25 + " L21关节映射探测 " + "=" * 25)
        print("将逐个测试关节索引 0 到 24。")
        print("按 Ctrl+C 随时中断测试。")
        print("=" * 72)

        print("首先，将所有关节移动到中立位置(128)...")
        hand.finger_move(pose=neutral_pose)
        time.sleep(3)  # 等待完全就位

        for joint_index in range(pose_length):
            expected_action = inferred_mapping.get(joint_index, "未知动作")
            print(f"\n--> 正在测试 Pose 索引: {joint_index} (预期动作: {expected_action})")

            if "预留" in expected_action:
                print("    索引为预留位，跳过。")
                continue

            # --- 测试最大位置 (255) ---
            print("    1. 移动到最大位置 (255)...")
            test_pose_max = neutral_pose.copy()
            test_pose_max[joint_index] = 255
            hand.finger_move(pose=test_pose_max)
            time.sleep(3)  # 增加等待时间以便观察

            # --- 测试最小位置 (0) ---
            print("    2. 移动到最小位置 (0)...")
            test_pose_min = neutral_pose.copy()
            test_pose_min[joint_index] = 0
            hand.finger_move(pose=test_pose_min)
            time.sleep(3)  # 增加等待时间以便观察

            # --- 每个关节测试完后，统一恢复到中立位 ---
            print("    3. 恢复到中立位置 (128)...")
            hand.finger_move(pose=neutral_pose)
            time.sleep(2)  # 等待恢复

    except KeyboardInterrupt:
        print("\n[信息] 用户中断了测试。")
    except Exception as e:
        print(f"\n[严重错误] 在程序运行过程中发生意外: {e}")
    finally:
        # --- 4. 最终清理 ---
        # 由于SDK没有提供shutdown()方法，我们能做的就是在退出前让手回到安全位置
        if hand:
            print("\n" + "=" * 25 + " 程序即将退出，执行清理 " + "=" * 25)
            try:
                print("--> 正在将手恢复到中立姿态...")
                hand.finger_move(pose=[128] * 25)
                time.sleep(2)
                print("✅ 已发送恢复姿态指令。")
            except Exception as e:
                print(f"[清理错误] 在恢复中立姿态时发生错误: {e}")

        print("\n[注意] SDK未提供关闭连接(shutdown)的方法，CAN总线可能未被优雅关闭。")
        print("如果下次运行时遇到问题，请使用 'sudo ip link' 命令或断电重启来复位。")
        print("程序已退出。")


if __name__ == "__main__":
    main()