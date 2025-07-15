#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray, Bool
from FP_robot_planning.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# 取得數字辨識結果
def num_callback(msg):
    global waypoint, putpoint
    # rospy.loginfo(f"傳入grasp.py之pos: {pos}")    
    first_two = int(f"{msg.data[0]}{msg.data[1]}")
    waypoint = str(first_two)
    last_two = int(f"{msg.data[2]}{msg.data[3]}")
    putpoint = str(last_two)
    # print("辨識到的位置:", msg.data)

# 訂閱假topic，為了讓service能一直工作
def fake_num_callback(msg):
    global waypoint, putpoint, grasp_done, pos, pos_home
    # rospy.loginfo(f"傳入grasp.py之pos: {pos}")    
    check_arm(pos)
    print("棧板位置:", waypoint, "放置位置:", putpoint)
    print("grasp_done:", grasp_done)
    if grasp_done == "準備歸貨":
        # 發布topic
        pos_home = False
        result_msg = Bool()
        result_msg.data = pos_home
        pos_pub.publish(result_msg)
        # 導航到棧板點
        navigate_to_waypoint(waypoint)

    elif grasp_done == "準備回歸":
        waypoint = "home"
        rospy.loginfo("回歸home")
        # 導航到原點
        navigate_to_waypoint(waypoint)
        # 發布topic
        pos_home = True  
        result_msg = Bool()
        result_msg.data = pos_home
        pos_pub.publish(result_msg)

def arm_grasp_client(pos): 
    rospy.wait_for_service('arm_grasp') 
    try: 
        arm_grasp = rospy.ServiceProxy('arm_grasp', ArmGrasp)
        resp1 = arm_grasp(pos) 
        rospy.loginfo(resp1.grasp)
        return resp1.grasp
    except rospy.ServiceException as e: 
        print("Service call failed: %s"%e)

# 用來啟動arm_grasp服務
def check_arm(pos):
    global pre_pos, grasp_done
    print("當前pos, pre_pos:",pos,pre_pos)
    # grasp為準備歸貨或準備回歸
    grasp = arm_grasp_client(pos)
    pre_pos = pos
    grasp_done = grasp

# 導航結果回調函數
def navigate_to_waypoint(wp):
    global pos   
    # 與move_base建立Action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server(rospy.Duration(5.0))
    # 目標點
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # 從waypoints獲取預設目標位置和方向
    waypoint = waypoints[wp]
    goal.target_pose.pose.position.x = waypoint['pos_x']
    goal.target_pose.pose.position.y = waypoint['pos_y']
    goal.target_pose.pose.position.z = waypoint['pos_z']
    goal.target_pose.pose.orientation.x = waypoint['ori_x']
    goal.target_pose.pose.orientation.y = waypoint['ori_y']
    goal.target_pose.pose.orientation.z = waypoint['ori_z']
    goal.target_pose.pose.orientation.w = waypoint['ori_w']
    client.send_goal(goal)
    client.wait_for_result()
    state = client.get_state()
    rospy.loginfo(f"導航狀態(3為done): {state}")

    # 如果到達目標點，會在往後移動1m，如直接導航至該點有原地碰撞之風險
    if(state == GoalStatus.SUCCEEDED):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # 從waypoints獲取預設目標位置和方向
        waypoint = waypoints[wp]
        goal.target_pose.pose.position.x = waypoint['pos_x']
        goal.target_pose.pose.position.y = waypoint['pos_y']-1
        goal.target_pose.pose.position.z = waypoint['pos_z']
        goal.target_pose.pose.orientation.x = waypoint['ori_x']
        goal.target_pose.pose.orientation.y = waypoint['ori_y']
        goal.target_pose.pose.orientation.z = waypoint['ori_z']
        goal.target_pose.pose.orientation.w = waypoint['ori_w']
        client.send_goal(goal)
        client.wait_for_result()
        state = client.get_state()
        rospy.loginfo("結束")
        # 到達後將pos改成對應位置
        pos = wp 


if __name__ == "__main__":    
    rospy.init_node("number_nav")
    # 定義原點、1號棧板點、2號棧板點、3號棧板點
    waypoints = {
    '1': {'pos_x': 0.5, 'pos_y': -1.6, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
    '2': {'pos_x': -1.5, 'pos_y': -1.6, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
    '3': {'pos_x': -3.5, 'pos_y': -1.6, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
    'home': {'pos_x': 4, 'pos_y': 1.1, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
    }
    # 用來與pos比較，決定是否需要使用arm_grasp service
    pre_pos = ""
    # 用來傳入當前位置給arm_grasp service
    pos = "home"
    # 導航的第幾個棧板點、棧板放置點
    waypoint = ""
    putpoint = ""
    # 儲存arm_grasp service回傳結果
    grasp_done = "準備歸貨"
    # 是否在home這個位置上，如果是的話要number_detect節點要進行數字辨識
    pos_home = True
    num_sub = rospy.Subscriber("/number_results", Int32MultiArray, num_callback, queue_size=1)
    rospy.Subscriber("/fake_number_results", Int32MultiArray, fake_num_callback, queue_size=1)
    pos_pub = rospy.Publisher('/pos_results', Bool, queue_size=10)
    rospy.spin()





# #!/usr/bin/env python3
# # coding=utf-8

# import numpy as np
# import rospy
# from std_msgs.msg import String, Int32MultiArray, Bool
# from geometry_msgs.msg import Twist
# import cv2
# import random
# import sys 
# from FP_robot_planning.srv import *

# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus

# def num_callback(msg):
#     global waypoint, putpoint
#     # rospy.loginfo(f"傳入grasp.py之pos: {pos}")    
#     first_two = int(f"{msg.data[0]}{msg.data[1]}")
#     waypoint = str(first_two)
#     last_two = int(f"{msg.data[2]}{msg.data[3]}")
#     putpoint = str(last_two)
#     # print("辨識到的位置:", msg.data)

    

# def fake_num_callback(msg):
#     global waypoint, putpoint, grasp_done, pos, pos_home, pre_pos
#     # rospy.loginfo(f"傳入grasp.py之pos: {pos}")    
#     check_arm(pos)
#     print("棧板位置:", waypoint, "放置位置:", putpoint)
#     print("grasp_done:", grasp_done)
#     if grasp_done == "準備歸貨":

#         pos_home = False
#         # 發布topic
#         result_msg = Bool()
#         result_msg.data = pos_home
#         pos_pub.publish(result_msg)
#         navigate_to_waypoint(waypoint)

#     elif grasp_done == "準備回歸":
#         waypoint = "home"
#         rospy.loginfo("回歸home")
#         navigate_to_waypoint(waypoint)
#         pos_home = True
#         # pre_pos = "歸貨點"    
#         # 發布topic
#         result_msg = Bool()
#         result_msg.data = pos_home
#         pos_pub.publish(result_msg)

# def arm_grasp_client(pos): 
#     rospy.wait_for_service('arm_grasp') 
#     try: 
#         arm_grasp = rospy.ServiceProxy('arm_grasp', ArmGrasp)
#         resp1 = arm_grasp(pos) 
#         rospy.loginfo(resp1.grasp)
#         return resp1.grasp
#     except rospy.ServiceException as e: 
#         print("Service call failed: %s"%e)

# def check_arm(pos):
#     # 用來啟動arm_grasp服務
#     global pre_pos, grasp_done
#     # 啟動條件(剛到達目標點時)
#     print("當前pos, pre_pos:",pos,pre_pos)
#     # grasp為準備歸貨或準備回歸
#     grasp = arm_grasp_client(pos)
#     pre_pos = pos
#     grasp_done = grasp

# # 導航結果回調函數
# def navigate_to_waypoint(wp):
#     global pos   
#     # 與move_base建立Action
#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     client.wait_for_server(rospy.Duration(5.0))
#     # cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     # # 消息
#     # twist = Twist()
#     # twist.linear.x = 0.2  # 向前移動，速度0.2m/s
#     # rate = rospy.Rate(10)  # 10Hz
#     # # 計算移動之時間，距離/速率
#     # move_duration = 1 / abs(twist.linear.x)
#     # # 取得停止時間
#     # move_time = rospy.Time.now() + rospy.Duration(move_duration)
#     # while rospy.Time.now() < move_time:
#     #     cmd_vel_pub.publish(twist) # 開始移動
#     #     rate.sleep()
#     # # 停止移動
#     # twist.linear.x = 0.0
#     # cmd_vel_pub.publish(twist)
#     # rospy.sleep(0.5)  # 停止後等待 1 秒

#     # # 旋轉設定 (向右旋轉 90 度)
#     # twist.angular.z = -0.5  # 角速度 (負值表示右轉)
#     # rotation_duration = 90.0 / (abs(twist.angular.z) * 180.0 / 3.141592)  # 角度/角速度，計算時間
#     # rotate_time = rospy.Time.now() + rospy.Duration(rotation_duration)

#     # # 開始旋轉
#     # while rospy.Time.now() < rotate_time:
#     #     cmd_vel_pub.publish(twist)
#     #     rate.sleep()

#     # # 停止旋轉
#     # twist.angular.z = 0.0
#     # cmd_vel_pub.publish(twist)
#     # rospy.sleep(0.5)  # 停止後等待 1 秒
    
#     # 目標點
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"  # 坐標系使用map
#     goal.target_pose.header.stamp = rospy.Time.now()
#     # 從waypoints獲取預設目標位置和方向
#     waypoint = waypoints[wp]
#     goal.target_pose.pose.position.x = waypoint['pos_x']
#     goal.target_pose.pose.position.y = waypoint['pos_y']
#     goal.target_pose.pose.position.z = waypoint['pos_z']
#     goal.target_pose.pose.orientation.x = waypoint['ori_x']
#     goal.target_pose.pose.orientation.y = waypoint['ori_y']
#     goal.target_pose.pose.orientation.z = waypoint['ori_z']
#     goal.target_pose.pose.orientation.w = waypoint['ori_w']
#     client.send_goal(goal)
#     client.wait_for_result()
#     state = client.get_state()
#     rospy.loginfo(f"導航狀態(3為done): {state}")
#     # 如果到達目標點，會在往後移動1m，如直接導航至該點有原地碰撞之風險
#     if(state == GoalStatus.SUCCEEDED):
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"  # 坐標系使用map
#         goal.target_pose.header.stamp = rospy.Time.now()
#         # 從waypoints獲取預設目標位置和方向
#         waypoint = waypoints[wp]
#         goal.target_pose.pose.position.x = waypoint['pos_x']
#         goal.target_pose.pose.position.y = waypoint['pos_y']-1
#         goal.target_pose.pose.position.z = waypoint['pos_z']
#         goal.target_pose.pose.orientation.x = waypoint['ori_x']
#         goal.target_pose.pose.orientation.y = waypoint['ori_y']
#         goal.target_pose.pose.orientation.z = waypoint['ori_z']
#         goal.target_pose.pose.orientation.w = waypoint['ori_w']
#         client.send_goal(goal)
#         client.wait_for_result()
#         state = client.get_state()
        
#         rospy.loginfo("結束")
#         # 到達後將pos改成對應位置
#         pos = wp 


# if __name__ == "__main__":    
#     rospy.init_node("number_nav")
#     waypoints = {
#     '1': {'pos_x': 0.5, 'pos_y': -1.8, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
#     '2': {'pos_x': -1.5, 'pos_y': -1.8, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
#     '3': {'pos_x': -3.5, 'pos_y': -1.8, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
#     'home': {'pos_x': 4, 'pos_y': 1.2, 'pos_z': 0.0, 'ori_x': 0.0, 'ori_y': 0.0, 'ori_z': 0.703554, 'ori_w': 0.710642},
#     }
#     # 用來與pos比較，決定是否需要使用arm_grasp service
#     pre_pos = ""
#     # 用來傳入當前位置給arm_grasp service
#     pos = "home"
#     # 導航的第幾個目標點
#     waypoint = ""
#     putpoint = ""
#     # 儲存arm_grasp service回傳結果
#     grasp_done = "準備歸貨"
#     # 是否在home這個位置上，如果是的話要數字辨識
#     pos_home = True
#     num_sub = rospy.Subscriber("/number_results", Int32MultiArray, num_callback, queue_size=1)
#     rospy.Subscriber("/fake_number_results", Int32MultiArray, fake_num_callback, queue_size=1)
#     # arm_sub = rospy.Subscriber("/arm_results", Bool, arm_callback, queue_size=1)
#     pos_pub = rospy.Publisher('/pos_results', Bool, queue_size=10)
#     rospy.sleep(3)
#     rospy.spin()







#     # --------------備份-----------


#     #!/usr/bin/env python3
#     # coding=utf-8

#     # import numpy as np
#     # import rospy
#     # from std_msgs.msg import String, Int32MultiArray, Bool
#     # import cv2
#     # import random
#     # import sys 
#     # from FP_robot_planning.srv import *

#     # import actionlib
#     # from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#     # def num_callback(msg):
#     #     global Nav_ing, grasp_done, waypoint
#     #     if grasp_done == "準備歸貨" and Nav_ing == False:
#     #         # rospy.loginfo("辨識到的位置: %s", msg.data[0, 1])
#     #         waypoint = '2'
#     #         navigate_to_waypoint('2')
#     #         # 設定Nav_ing為True，表示正在導航
#     #         Nav_ing = True

#     # # def arm_callback(msg):
#     # #     global waitForget
#     # #     waitForget = msg.data

#     # def arm_grasp_client(pos): 
#     #     rospy.wait_for_service('arm_grasp') 
#     #     global pre_pos
#     #     try: 
#     #         if pre_pos != pos:
#     #             arm_grasp = rospy.ServiceProxy('arm_grasp', ArmGrasp)
#     #             resp1 = arm_grasp(pos) 
#     #             pre_pos = pos
#     #             rospy.loginfo(resp1.grasp)
#     #             return resp1.grasp
#     #     except rospy.ServiceException as e: 
#     #         print("Service call failed: %s"%e)


#     # # 導航結果回調函數
#     # def navigate_to_waypoint(wp):
#     #     rospy.loginfo(wp)
#     #     msg = String()
#     #     msg.data = '2'
#     #     navi_pub.publish(msg)
#     #     rospy.loginfo("結束")
        
#     # def resultNavi(msg):
#     #     global pos, Nav_ing, waypoint
#     #     rospy.loginfo("导航结果 = %s",msg.data)
#     #     if(msg.data=="done"):
#     #         if(pos != "home"):
#     #             # 创建一个 action 客户端，使用 move_base 发送目标
#     #             client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     #             client.wait_for_server()
#     #             # 设置目标点
#     #             goal = MoveBaseGoal()
#     #             goal.target_pose.header.frame_id = "base_footprint"  # 使用机器人基坐标系
#     #             goal.target_pose.header.stamp = rospy.Time.now()
#     #             goal.target_pose.pose.position.x = -1.0  # 向后移动一米
#     #             goal.target_pose.pose.orientation.w = 1.0  # 保持当前朝向
#     #             # 发送目标点到 move_base 并等待完成
#     #             client.send_goal(goal)
#     #             wait = client.wait_for_result()
#     #             Nav_ing = False
#     #             pos = waypoint
#     #         else:
#     #             # 创建一个 action 客户端，使用 move_base 发送目标
#     #             client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     #             client.wait_for_server()
#     #             # 设置目标点
#     #             goal = MoveBaseGoal()
#     #             goal.target_pose.header.frame_id = "base_footprint"  # 使用机器人基坐标系
#     #             goal.target_pose.header.stamp = rospy.Time.now()
#     #             goal.target_pose.pose.position.y = -1.0  # 向后移动一米
#     #             goal.target_pose.pose.orientation.w = 1.0  # 保持当前朝向
#     #             # 发送目标点到 move_base 并等待完成
#     #             client.send_goal(goal)
#     #             wait = client.wait_for_result()
#     #             Nav_ing = False
#     #             pos = waypoint

#     # if __name__ == "__main__":
#     #     pre_pos = ""
#     #     pos = "home"
#     #     Nav_ing = False
#     #     waypoint = ""
#     #     rospy.init_node("demo_map_tools")
#     #     grasp_done = arm_grasp_client(pos)
#     #     # 发布航点名称话题
#     #     navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10)
#     #     # 订阅导航结果话题
#     #     result_sub = rospy.Subscriber("/waterplus/navi_result", String, resultNavi, queue_size=10)
#     #     num_sub = rospy.Subscriber("/number_results", Int32MultiArray, num_callback, queue_size=1)
#     #     # arm_sub = rospy.Subscriber("/arm_results", Bool, arm_callback, queue_size=1)
        
#     #     # 延时1秒钟，让后台的话题发布操作能够完成
#     #     rospy.sleep(3)
#     #     rospy.spin()