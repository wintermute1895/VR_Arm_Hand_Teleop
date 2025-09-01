# debug_from_pico_sample.py

import json
import numpy as np

from config import Config
from pico_data_handler import PICODataHandler
from robothandmodel import RobotHandModel
from retargeting import HandOptimizer
from fk import HandForwardKinematics

def main():
    # 1. 初始化所有模块
    config = Config()
    pico_handler = PICODataHandler()
    robot_model = RobotHandModel(config)
    optimizer = HandOptimizer(robot_model, config)
    fk_calc = HandForwardKinematics(config)

    # 2. 从文件加载一帧真实的PICO数据
    input_filepath = 'pico_sample_data.json'
    try:
        with open(input_filepath, 'r') as f:
            pico_frame_data = json.load(f)
    except FileNotFoundError:
        print(f"错误: PICO样本数据文件 '{input_filepath}' 未找到。")
        return

    # 3. 使用PICO数据处理器进行转换
    print(f"--- 正在处理 '{input_filepath}' ---")
    w_lhs = pico_handler.process_pico_data(pico_frame_data)
    print("PICO数据成功映射并转换为w_lhs。")

    # 4. 校准与重定向
    print("\n--- 正在进行校准与重定向 ---")
    robot_model.calibrate(w_lhs)
    optimized_q_dict, final_loss = optimizer.optimize_q(w_lhs)

    v_rhs = robot_model.v
    fk_rhs = fk_calc.calculate_fk_all_keypoints(optimized_q_dict)

    # 5. [核心修复] 保存结果供可视化，现在包含 scale_factor
    output_data = [{
        'w_lhs': w_lhs.tolist(),
        'v_rhs': v_rhs.tolist(),
        'fk_rhs': fk_rhs.tolist(),
        'q_dict': optimized_q_dict,
        'loss': final_loss,
        'scale_factor': robot_model.scale_factor # <--- 新增
    }]
    output_filename = 'debug_output.json'
    with open(output_filename, 'w') as f:
        json.dump(output_data, f, indent=2)

    print(f"\n--- 单帧调试完成 ---")
    print(f"最终损失: {final_loss:.4f}")
    print(f"尺度因子: {robot_model.scale_factor:.3f}")
    print(f"结果已保存到 '{output_filename}'，请运行 visualization.py 查看。")

if __name__ == "__main__":
    main()