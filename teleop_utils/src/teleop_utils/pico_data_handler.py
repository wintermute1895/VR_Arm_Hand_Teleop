# pico_data_handler.py (最终修正版)

import numpy as np
from config import Config


class PICODataHandler:
    def __init__(self):
        self.config = Config()

        # PICO ID 到我们模型名称的映射 (保持不变)
        self.pico_to_our_map = {
            1: 'wrist',
            2: 'thumb_mcp', 3: 'thumb_pip', 4: 'thumb_dip', 5: 'thumb_tip',
            7: 'index_mcp', 8: 'index_pip', 9: 'index_dip', 10: 'index_tip',
            12: 'middle_mcp', 13: 'middle_pip', 14: 'middle_dip', 15: 'middle_tip',
            17: 'ring_mcp', 18: 'ring_pip', 19: 'ring_dip', 20: 'ring_tip',
            22: 'little_mcp', 23: 'little_pip', 24: 'little_dip', 25: 'little_tip',
        }

    def process_pico_data(self, raw_pico_frame_data):
        """
        接收一帧原始的PICO数据(字典列表)，返回一个标准的(22, 3) w_lhs 数组。
        """
        # --- [核心修正] ---
        # 根据C#脚本发来的真实数据结构进行解析
        # C# 发送的键是 'id', 'posX', 'posY', 'posZ'
        pico_points = {
            joint['id']: np.array([joint['posX'], joint['posY'], joint['posZ']])
            for joint in raw_pico_frame_data if joint
        }
        # --- 修正结束 ---

        w_lhs = np.zeros((len(self.config.KEYPOINT_MAP), 3))

        for pico_id, our_name in self.pico_to_our_map.items():
            if pico_id in pico_points:
                our_idx = self.config.KEYPOINT_MAP[our_name]
                w_lhs[our_idx] = pico_points[pico_id]

        mcp_names = ['index_mcp', 'middle_mcp', 'ring_mcp', 'little_mcp']
        mcp_indices = [self.config.KEYPOINT_MAP[name] for name in mcp_names]
        w_lhs[self.config.KEYPOINT_MAP['palm_center']] = np.mean(w_lhs[mcp_indices], axis=0)

        return w_lhs * 1000.0  # 转换单位：米 -> 毫米