# msg/UnifiedDataPacket.msg
std_msgs/Header header

# 手柄位姿 (右手柄)
geometry_msgs/Point controller_pos
geometry_msgs/Quaternion controller_rot

# 手柄按钮状态 (右手柄)
bool primary_btn     # X/A 键
bool secondary_btn   # B/Y 键
float32 trigger_val    # 扳机键按压程度 (0.0-1.0)
float32 grip_val       # 抓握键按压程度 (0.0-1.0)
float32 axis_vec2_x    # 摇杆X轴值 (-1.0-1.0)
float32 axis_vec2_y    # 摇杆Y轴值 (-1.0-1.0)
bool menu_btn         # 菜单键

# 裸手关节点数据 (右手)
teleop_nodes/JointData[] right_hand_joints