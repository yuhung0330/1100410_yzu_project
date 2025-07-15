#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # 機械臂允許誤差值
        arm.set_goal_joint_tolerance(0.001)
        
        # 機械臂原始狀態
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)
         
        # 設置機械臂的目標位置，使用七軸的位置數據進行描述（單位：弧度）
        joint_positions = [0.000, 0.122, 0.928, 1.222, 0.880, 0.000, 0.000]
        arm.set_joint_value_target(joint_positions)
                 
        # 控制機械臂完成運動
        arm.go()
        rospy.sleep(1)
        
        # 關閉並退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
