#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 导入我们需要的“工具箱”
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

print("Arm Controller is ready.")

class ArmController:
    def __init__(self):
        """
        这是我们控制器的“构造函数”，当创建一个控制器实例时，
        这里的代码会首先被执行。
        """
        # 1. 初始化moveit_commander。这是每次使用它都必须做的第一件事。
        moveit_commander.roscpp_initialize(sys.argv)

        # 2. 初始化ROS节点。
        #    'my_arm_controller_node' 是我们给这个节点取的名字。
        #    anonymous=True 保证了每次启动时节点名都是唯一的，避免冲突。
        rospy.init_node('my_arm_controller_node', anonymous=True)

        # 3. 创建MoveGroupCommander实例。这是最重要的对象！
        #    我们通过它来发送所有规划和运动的指令。
        #    "arm" 是我们在MoveIt!配置里定义的“规划组”的名字。
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # 打印一条日志，确认初始化成功。
        rospy.loginfo("Arm Controller has been initialized and is ready.")
        rospy.loginfo("Controlling planning group: %s", self.move_group.get_name())

    def go_to_pose_goal(self, x, y, z):
        """
        规划并移动机械臂末端到一个指定的笛卡尔空间坐标。
        """
        rospy.loginfo("Planning to move to Pose Goal: [x=%.2f, y=%.2f, z=%.2f]", x, y, z)

        # 创建一个Pose消息对象
        pose_goal = Pose()
        
        # 我们只关心位置，姿态可以先用一个简单的默认值（末端垂直向下）
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        # 1. 设置目标
        self.move_group.set_pose_target(pose_goal)

        # 2. 规划并执行 (go() = plan + execute)
        success = self.move_group.go(wait=True)

        # 3. 清理工作，非常重要！
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            rospy.loginfo("Movement to pose goal successful.")
        else:
            rospy.logerr("Movement to pose goal failed!")
        
        return success
    
    def go_to_home_position(self):
        """
        规划并移动机械臂到一个预定义的“家”位置。
        """
        rospy.loginfo("Planning to return to Home Position.")

        # 使用在SRDF文件中预定义的位姿名称。
        # "zero" 是睿尔曼功能包里为机械臂定义的零位姿态。
        self.move_group.set_named_target("zero")

        # 规划并执行
        self.move_group.go(wait=True)

        # 清理
        self.move_group.stop()

        rospy.loginfo("Successfully returned to Home Position.")

if __name__ == '__main__':
    try:
        # 1. 创建我们遥控器的一个实例（对象）
        controller = ArmController()

        # 2. 编写我们的“剧本”，让用户通过按回车来触发每一步
        input("============ Press `Enter` to move to the first goal (0.3, 0.2, 0.5) ...")
        controller.go_to_pose_goal(0.3, 0.2, 0.5)

        input("============ Press `Enter` to move to the second goal (0.3, -0.2, 0.5) ...")
        controller.go_to_pose_goal(0.3, -0.2, 0.5)

        input("============ Press `Enter` to return home ...")
        controller.go_to_home_position()

        rospy.loginfo("Mission accomplished. All movements complete.")

    except rospy.ROSInterruptException:
        # 捕获ROS关闭时（如Ctrl+C）的异常
        pass
    except KeyboardInterrupt:
        # 捕获用户按Ctrl+C的异常
        pass
    finally:
        # 无论成功还是失败，最后都确保关闭moveit_commander
        rospy.loginfo("Shutting down the controller.")
        moveit_commander.roscpp_shutdown()