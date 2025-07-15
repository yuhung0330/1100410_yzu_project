#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory, PlanningScene, OrientationConstraint
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

        # scene = PlanningSceneInterface()
        # scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        # mesh_name = "conveyor"
        # mesh_pose = PoseStamped()
        # mesh_pose.header.frame_id = "map"
        # mesh_pose.pose.position.x = 8.0  # 根據你的Gazebo模型的實際位置調整
        # mesh_pose.pose.position.y = -5.0
        # mesh_pose.pose.position.z = 0.0
        # q = quaternion_from_euler(0, 0, 1.5708)  # 90度 = 1.5708 弧度
        # mesh_pose.pose.orientation.x = q[0]
        # mesh_pose.pose.orientation.y = q[1]
        # mesh_pose.pose.orientation.z = q[2]
        # mesh_pose.pose.orientation.w = q[3]
        # mesh_file_path = "/home/yuhung/catkin_ws/src/conveyor2_description/meshes/base_link.stl"
        # size = [0.001, 0.001, 0.001]
        # scene.add_mesh(mesh_name, mesh_pose, mesh_file_path, size)  
        # # 確保模型加入到碰撞場景
        # rospy.sleep(1)      
        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.move_group.MoveGroupCommander("arm")
        gripper = moveit_commander.MoveGroupCommander('gripper')
        end_effector_link = arm.get_end_effector_link()   
        # arm.set_planner_id("FMT")
        # 获取终端link的名称
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
        arm.set_planning_time(5.0)
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 機械臂允許誤差值
        arm.set_goal_position_tolerance(0.0001)
        arm.set_goal_orientation_tolerance(0.0001)
        
        # 機械臂原始狀態
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(2)
        
        # 設置機械臂工作空間中的目標位置，位置使用 x、y、z 坐標描述
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now() 
        # position:
        #     x: -1.1336686623934829
        #     y: -0.041596549539601124
        #     z: 1.0115731141518427
        # orientation:
        #     x: 6e-05
        #     y: 0.999987
        #     z: 0.0
        #     w: 0.005107

        # x: -0.6209023229752435
        # y: -0.5332674919276206
        # z: 1.0115244917079134
        # orientation:
        # x: -0.6902485279320676
        # y: 0.7235547590192297
        # z: 0.003513143357055963
        # w: 0.0036246671558945102
        
        # x: -1.1336686623934829
        # y: -0.0041596549539601124
        # z: 1.0115731141518427
        target_pose.pose.position.x = -0.6046
        target_pose.pose.position.y = -0.000032158
        target_pose.pose.position.z = 2.1
        target_pose.pose.orientation.x = -0.0079162
        target_pose.pose.orientation.y = -0.0000081464
        target_pose.pose.orientation.z = 0.99995
        target_pose.pose.orientation.w = -0.0059

        # target_pose.pose.position.x = -1.2705943953901795
        # target_pose.pose.position.y = -0.0080996599467076
        # target_pose.pose.position.z = 1.0113456030650472
        # target_pose.pose.orientation.x = 0.000060
        # target_pose.pose.orientation.y = 0.999987
        # target_pose.pose.orientation.z = 0.000000
        # target_pose.pose.orientation.w = 0.005107

        # 設定現在位置為機械臂原始狀態
        arm.set_start_state_to_current_state()
        
        # 設定機械臂目標位置
        arm.set_pose_target(target_pose, end_effector_link)

        # 規劃路徑
        plan_success, traj, planning_time, error_code = arm.plan()
        print(type(traj))
        
        # 根據路徑移動
        arm.execute(traj)
        rospy.sleep(1)

        # 添加方向约束
        # oc = OrientationConstraint()
        # oc.header.frame_id = end_effector_link
        # oc.link_name = end_effector_link
        # oc.orientation = Quaternion(0.0000000, 0.999990, 0.0000000, 0.0000000)
        # oc.absolute_x_axis_tolerance = 3.250000  # 你可以根据需要调整这些值
        # oc.absolute_y_axis_tolerance = 0.000100
        # oc.absolute_z_axis_tolerance = 3.250000
        # oc.weight = 1.0
        # constraints = moveit_commander.Constraints()
        # constraints.orientation_constraints.append(oc)
        # arm.set_path_constraints(constraints)


 

if __name__ == "__main__":
    MoveItIkDemo()
