#!/usr/bin/env python3
import rospy
import tf2_ros
import sys

def get_robot_end_effector_pose():
    """
    连接到ROS TF系统，查询并打印机器人末端执行器相对于基座的位姿。
    """
    rospy.init_node('get_robot_pose_node', anonymous=True)

    # 创建一个TF buffer和listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # --- 参数 (请根据您的机器人模型确认) ---
    base_frame = 'base_link'  # 机器人的根坐标系
    end_effector_frame = 'Link6' # 您想要查询位姿的末端连杆

    rospy.loginfo(f"Attempting to get transform from '{base_frame}' to '{end_effector_frame}'...")
    rospy.loginfo("Please make sure your robot's state publisher is running.")

    transform = None
    try:
        # 等待TF数据可用，最多等待5秒
        # lookup_transform('目标坐标系', '源坐标系', ...)
        transform = tf_buffer.lookup_transform(base_frame, end_effector_frame, rospy.Time(0), rospy.Duration(5.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"Failed to get transform: {e}")
        rospy.logerr("Is robot_state_publisher running and publishing TF?")
        sys.exit(1)

    # 清晰地打印获取到的位姿信息
    pos = transform.transform.translation
    quat = transform.transform.rotation

    print("\n" + "="*50)
    print(f"Pose of '{end_effector_frame}' relative to '{base_frame}':")
    print("\n--- Position (for robot_pos list) ---")
    print(f"[{pos.x:.8f}, {pos.y:.8f}, {pos.z:.8f}]")
    
    print("\n--- Orientation Quaternion (for robot_quat list) ---")
    print(f"[{quat.x:.8f}, {quat.y:.8f}, {quat.z:.8f}, {quat.w:.8f}]")
    print("\n" + "="*50)
    print("You can now copy these lists into your calibration script.")

if __name__ == '__main__':
    # 在运行此脚本前，请确保 roscore 和您机器人的驱动/状态发布器正在运行
    # 例如: roslaunch direct_teleop backend_only.launch
    get_robot_end_effector_pose()