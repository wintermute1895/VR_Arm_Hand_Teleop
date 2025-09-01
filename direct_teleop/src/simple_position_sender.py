#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time

def send_test_positions():
    rospy.init_node('simple_position_sender', anonymous=True)

    # 1. 机械臂的7个关节名称 (确保和你的YAML文件一致)
    joint_names = [f'joint{i+1}' for i in range(7)]

    # 2. 创建7个Publisher，每个对应一个关节控制器
    publishers = []
    for joint_name in joint_names:
        topic_name = f'/arm/{joint_name}_position_controller/command'
        pub = rospy.Publisher(topic_name, Float64, queue_size=10)
        publishers.append(pub)
    
    rospy.loginfo("Waiting for publishers to connect...")
    # 等待一小段时间，确保 publisher 和 subscriber 建立连接
    rospy.sleep(1.0) 

    # 3. 定义一个目标姿态 (7个关节的目标角度，单位是弧度)
    # 这个姿态会让机械臂稍微抬起和弯曲
    target_positions = [0.0, 0.5, 0.5, 0.0, -0.5, 0.0, 0.0]

    if len(target_positions) != len(publishers):
        rospy.logerr("Mismatch between number of target positions and publishers!")
        return

    # 4. 发布指令
    rospy.loginfo(f"Publishing target positions: {target_positions}")
    for i in range(len(publishers)):
        publishers[i].publish(Float64(target_positions[i]))
        
    rospy.loginfo("Goal positions published. Script will exit in 5 seconds.")
    rospy.sleep(5.0) # 保持发布一段时间，然后退出

if __name__ == '__main__':
    try:
        send_test_positions()
    except rospy.ROSInterruptException:
        pass