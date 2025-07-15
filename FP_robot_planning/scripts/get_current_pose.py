#! /usr/bin/env python3
#coding=utf-8
import cv2
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import numpy as np
import sys
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory, PlanningScene, OrientationConstraint, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty
from FP_robot_planning.srv import ArmGrasp, ArmGraspResponse 


class graspDemo:
    def __init__(self):
        # 初始化moveit控制器
        moveit_commander.roscpp_initialize(sys.argv)
        # moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.robot.RobotCommander()
        # 初始化manipulator group
        self.arm = moveit_commander.move_group.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.end_effector_link = self.arm.get_end_effector_link()   
    
        # 機械手臂參考座標系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        
        # 誤差值
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.gripper.set_goal_joint_tolerance(0.001)
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(10.0)
        #PlanningScene碰撞
        # self.scene = PlanningSceneInterface()
        # self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        # self.mesh_name = "conveyor"
        # self.mesh_pose = PoseStamped()
        # self.mesh_pose.header.frame_id = "map"
        # self.mesh_pose.pose.position.x = 8.0
        # self.mesh_pose.pose.position.y = -5.0
        # self.mesh_pose.pose.position.z = 0.0
        # q = quaternion_from_euler(0, 0, 1.5708)  # 90度 = 1.5708 弧度
        # self.mesh_pose.pose.orientation.x = q[0]
        # self.mesh_pose.pose.orientation.y = q[1]
        # self.mesh_pose.pose.orientation.z = q[2]
        # self.mesh_pose.pose.orientation.w = q[3]
        # mesh_file_path = "/home/yuhung/catkin_ws/src/conveyor2_description/meshes/base_link.stl"
        # size = [0.001, 0.001, 0.001]
        # self.scene.add_mesh(self.mesh_name, self.mesh_pose, mesh_file_path, size)  
        # rospy.sleep(1)      
            
        # # 儲存物體顏色的字典
        # self.colors = dict()
        # box1_id = 'box1'
        # box1_size = [0.4, 0.3, 0.27]
        # box1_pose = PoseStamped()
        # box1_pose.header.frame_id = "map"
        # box1_pose.pose.position.x = 8
        # box1_pose.pose.position.y = -4.2
        # box1_pose.pose.position.z = 0.8
        # box1_pose.pose.orientation.w = 1.0   
        # self.scene.add_box(box1_id, box1_pose, box1_size)
        # self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        # # 發布場景顏色
        # self.sendColors()
    def move(self, pose, orientation):
        # 機械臂移動控制 pose 位置 orientation 方向
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = pose[0]
        target_pose.pose.position.y = pose[1]
        target_pose.pose.position.z = pose[2]
        target_pose.pose.orientation.x = orientation[0]
        target_pose.pose.orientation.y = orientation[1]
        target_pose.pose.orientation.z = orientation[2]
        target_pose.pose.orientation.w = orientation[3]
        print("Target Pose:", target_pose)
        # 設定現在位置為機械臂原始狀態
        self.arm.set_start_state_to_current_state()
        # 設定機械臂目標位置
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        # 規劃路徑
        plan_success, traj, planning_time, error_code = self.arm.plan()
        print(type(traj))
        # 根據路徑移動
        self.arm.execute(traj)
        rospy.sleep(1)

    def go_named(self, place):
        # 移動至命名的pose
        self.arm.set_named_target(place)
        self.arm.go(wait=True)

    def get_it(self):
        #夾爪夾取
        self.gripper.set_joint_value_target([0.16, -0.16])
        self.gripper.go()
        # print("已抓取")
        rospy.sleep(1)

    def put_it(self):
        #夾爪放開
        global findObject
        self.gripper.set_joint_value_target([0, 0])
        self.gripper.go()
        findObject = False
        rospy.sleep(1)
        
    # def setColor(self, name, r, g, b, a = 0.9):
    #     # 初始化moveit顏色對象
    #     color = ObjectColor()
        
    #     # 設置顏色值
    #     color.id = name
    #     color.color.r = r
    #     color.color.g = g
    #     color.color.b = b
    #     color.color.a = a
        
    #     # 更新顏色字典
    #     self.colors[name] = color 

    # # 將顏色設置發送並應用到moveit場景當中
    # def sendColors(self):
    #     # 初始化規劃場景對象
    #     p = PlanningScene()

    #     # 需要設置規劃場景是否有差異     
    #     p.is_diff = True
        
    #     # 從顏色字典中取出顏色設置
    #     for color in self.colors.values():
    #         p.object_colors.append(color)
        
    #     # 發布場景物體顏色設置
    #     self.scene_pub.publish(p)
            
    @property
    def poseNow(self):
        # 返回當前座標
        return self.arm.get_current_pose(self.end_effector_link)
    
    def move_by_posNow(self, dx, dy, dz):
        # 在當前座標的基礎上設置目標
        self.arm.set_start_state_to_current_state()
        
        pos = self.poseNow.pose
        pos.position.x -= dx
        pos.position.y -= dy
        pos.position.z -= dz
        
        self.arm.set_pose_target(pos, self.end_effector_link)
        self.arm.go(wait=True)
        
    def move_by_joints(self,joint):
        # 設置關節座標
        current_joint_positions = self.arm.get_current_joint_values()
        print(current_joint_positions)
        if(current_joint_positions[0]>3.14):
            current_joint_positions[0] -= joint
        else:
            current_joint_positions[0] += joint
        print(current_joint_positions)
        self.arm.set_joint_value_target(current_joint_positions)
        self.arm.go(wait=True)
        # 向下移動以準備放置物體
        grasp_demo.move_cartesian_byNow(0, 0 , 0.15)
        # grasp_demo.go_named('put')

        # 夾爪放開
        # global findObject
        # self.gripper.set_joint_value_target([0, 0])
        # self.gripper.go()
        # findObject = False
        
        return("準備歸貨")
        
        global waitForget
        waitForget = False
        
     
    def move_cartesian(self, x, y, z):    
        # 笛卡爾規劃
        waypoints = []
        pos = self.poseNow.pose
        pos.position.x = x
        pos.position.y = y
        pos.position.z = z
        waypoints.append(pos)
        fraction = 0.0 
        maxtries = 100
        attempts = 0
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路點列表
                                    0.1,        # eef_step，終端步進值
                                    0.0,         # jump_threshold，跳躍閾值
                                    True)        # 避障規劃
            attempts += 1
            
        if fraction == 1.0:
            self.arm.execute(plan)
            return True
        else:
            pass
        return False
    

    def move_cartesian_byNow(self, x, y, z):    
        # 笛卡爾規劃（從當前座標設置目標）
        waypoints = []
        pos = self.poseNow.pose
    #        waypoints.append(pos)
        pos.position.x -= x
        pos.position.y -= y
        pos.position.z -= z
        # pos.orientation.x = preState['down'][0]
        # pos.orientation.y = preState['down'][1]
        # pos.orientation.z = preState['down'][2]
        # pos.orientation.w = preState['down'][3]
        waypoints.append(pos)
        fraction = 0.0 
        maxtries = 100
        attempts = 0
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路點列表
                                    0.1,        # eef_step，終端步進值
                                    0.0,         # jump_threshold，跳躍閾值
                                    True)        # 避障規劃
            attempts += 1
            # print(fraction, attempts)
            
        if fraction == 1.0:
            self.arm.execute(plan)
            return True
        else:
            pass
        return False
                
    def shutDown(self):
        # moveit关闭
        moveit_commander.roscpp_initializer.roscpp_shutdown()
        rospy.loginfo('grasp complete!!!')

def get_rotation_matrix(q):
    # 在TF中，它是xyzw
    # xyzw方向轉旋轉矩陣
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    
    rot = [[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], 
            [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x],
            [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]]
    return rot

def get_CameraFrame_Pos(u, v, depthValue):
    # 圖像系轉相機系（u、v圖像座標，depthValue對應座標的深度值）
    # fx fy cx cy 為相機內參
    fx = 343.49636753580074
    fy = 343.49636753580074
    cx = 320
    cy = 240

    z = float(depthValue)
    y = float((u - cx) * z) / fx    #圖片長對應相機坐標系y軸
    x = float((v - cy) * z) / fy    #圖片寬對應相機坐標系x軸
    return [x, -y, z, 1]    #相機坐標系y軸與base_link正負相反

def get_RT_matrix(base_frame, reference_frame):
    # 獲取base_frame到reference_frame的旋轉平移矩陣，通過tf變換獲取
    listener = tf.TransformListener()
    i = 3 # 嘗試3次，三次未獲取到則獲取失敗
    while i != 0:
        try:
            listener.waitForTransform(base_frame, reference_frame, rospy.Time(0), rospy.Duration(3.0))
            camera2World = listener.lookupTransform(base_frame, reference_frame, rospy.Time(0))
            break
        except:
            print("獲取變換錯誤，嘗試下一次")
            pass
        i = i - 1
    
    T = camera2World[0]
    R = get_rotation_matrix(camera2World[1])
    
    R[0].append(0)
    R[1].append(0)
    R[2].append(0)
    R.append([0.0, 0.0, 0.0, 1.0])
    R = np.mat(R)
    return [R, T]

def coordinate_transform(cameraFrame_pos, R, T):
    # 相機系轉base_link座標系，先旋轉再平移
    worldFrame_pos = R.I * np.mat(cameraFrame_pos).T 
    
    worldFrame_pos[0,0] = worldFrame_pos[0,0] + T[0]
    worldFrame_pos[1,0] = worldFrame_pos[1,0] + T[1]
    worldFrame_pos[2,0] = worldFrame_pos[2,0] + T[2]
    worldFrame_pos = [worldFrame_pos[0,0], worldFrame_pos[1,0], worldFrame_pos[2,0]]
    # print(worldFrame_pos)
    return worldFrame_pos

def getPutPosition(data):
    global findObject, u, v, pos
    bridge = CvBridge()
    # 處理/image_raw的圖片，轉換成bgr8 cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Define color range for the box (adjust based on the box color)
    lower_orange = np.array([10, 100, 100])  # 橘色的下界 
    upper_orange = np.array([25, 255, 255])  # 橘色的上界

    # Threshold the image to get only the box color
    mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour, assuming it is the box
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Draw the contour and the center
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            u = int(M["m10"] / M["m00"])
            v = int(M["m01"] / M["m00"])
            cv2.circle(cv_image, (u, v), 5, (0, 255, 0), -1)
            findObject = True
        cv2.drawContours(cv_image, [largest_contour], -1, (255, 0, 0), 2)

    cv2.imshow('Detected Box', cv_image)
    cv2.waitKey(1)


def num_callback(msg):
    global pos, put
    if pos == "home":
        last_two = int(f"{msg.data[2]}{msg.data[3]}")
        put = str(last_two)
        rospy.loginfo("放的位置: %s", put)

def depthCallback(data):
    # 深度圖像回調函數
    global depthImg,depthOK
    depthOK = False
    depthImg = CvBridge().imgmsg_to_cv2(data, data.encoding)
    # print(depthImg[v,u])
    depthOK = True
def grasp_service(req):
    if req.pos == "home":
        global u, v    
        rospy.loginfo('開始掃描')
        global depthOK, findObject
        # 機械臂移動到該位置
        grasp_demo.go_named('scan')  # 移動到可以找到目標的位置
        rospy.loginfo('結束掃描')
        # 如果檢測到待抓取物體且獲取到了深度圖
        if depthOK and findObject:
            # 獲取相機系到世界系的旋轉平移矩陣
            [R, T] = get_RT_matrix('base_link', 'camera2_1')
            u, v = int(u), int(v)  # 防止檢測的座標越界
            # print(depthImg[v,u])
            # 圖像座標轉相機系座標
            # print(u, v, depthImg[v,u])
            cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u])
            
            # 相機系座標轉世界系座標
            worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
            # print(graspObject + ' pose is: ', worldFrame_pos)
            
            # 移動到抓取物體的上方
            grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.25],endDown)
            rospy.sleep(0.5) # wait for grasp

            # 向下移動以準備抓取物體
            grasp_demo.move_cartesian_byNow(0, 0 , 0.10) # avoid grasp other object by mistake
            # 夾爪抓取
            grasp_demo.get_it()
            # 向上移動
            grasp_demo.move_cartesian_byNow(0, 0 , -0.35)
            rospy.sleep(0.5)
            # 利用joint0製造平移路徑
            result = grasp_demo.move_by_joints(3.14)
            return ArmGraspResponse(result)
    else:
        grasp_demo.go_named('put')
        grasp_demo.put_it()
        print("put")
        return ArmGraspResponse("準備回歸")

def arm_grasp_server():
    s = rospy.Service('add_two_ints', ArmGrasp, grasp_service)

# 吸盤向下對應的 xyzw 座標
# endDown = [0.013623, 0.999826, -0.000042, 0.0127391]
# endDown = [0.9997, 0.000026, 0.0011835, 0.0000018]
endDown = [1, 0, 0, 0]

if __name__ == "__main__":
    # 初始化 ROS 節點
    rospy.init_node('grasp')
    # 深度圖像和檢測框對應的 topic 名稱
    depth_topic = '/camera/depth/image_raw'
    color_topic = '/camera/color/image_raw'
    # 實例化抓取控制類
    grasp_demo = graspDemo()
    grasp_demo.go_named('home') 
    # 初始化圖像系座標及待抓取物體
    u = 320
    v = 240    
    findObject = False
    depthOK = False
    # 訂閱深度圖像和檢測框對應的 topic
    rospy.Subscriber(depth_topic, Image, depthCallback, queue_size=1)
    rospy.Subscriber(color_topic, Image, getPutPosition, queue_size=1)
    rospy.loginfo('開始掃描')
    # 機械臂移動到該位置
    grasp_demo.go_named('scan')  # 移動到可以找到目標的位置
    rospy.loginfo('結束掃描')
    # 如果檢測到待抓取物體且獲取到了深度圖
    if depthOK and findObject:
        # 獲取相機系到世界系的旋轉平移矩陣
        [R, T] = get_RT_matrix('base_link', 'camera2_1')
        u, v = int(u), int(v)  # 防止檢測的座標越界
        # print(depthImg[v,u])
        # 圖像座標轉相機系座標
        print(u, v, depthImg[v,u])
        cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u])
        print("cameraFrame_pos:", cameraFrame_pos)
        # 相機系座標轉世界系座標
        worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
        # print(graspObject + ' pose is: ', worldFrame_pos)

        # 移動到抓取物體的上方
        grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.25],endDown)
        rospy.sleep(0.5) # wait for grasp

        # 向下移動以準備抓取物體
        grasp_demo.move_cartesian_byNow(0, 0 , 0.1) # avoid grasp other object by mistake
        # 夾爪抓取
        grasp_demo.get_it()
        # 向上移動
        grasp_demo.move_cartesian_byNow(0, 0 , -0.35)
        rospy.sleep(0.5)
        # 利用joint0製造平移路徑
        grasp_demo.go_named('put')

    rospy.spin ()
    # while not rospy.is_shutdown():
    #     # 如果檢測到待抓取物體且獲取到了深度圖
    #     if depthOK and findObject and waitForget:
    #         # 獲取相機系到世界系的旋轉平移矩陣
    #         [R, T] = get_RT_matrix('base_link', 'camera2_1')
    #         u, v = int(u), int(v)  # 防止檢測的座標越界
    #         # print(depthImg[v,u])
    #         # 圖像座標轉相機系座標
    #         cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u])
    #         # print(u, v, depthImg[v,u])
    #         # 相機系座標轉世界系座標
    #         worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
    #         # print(graspObject + ' pose is: ', worldFrame_pos)
            
    #         # 移動到抓取物體的上方
    #         grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.25],endDown)
    #         rospy.sleep(0.5) # wait for grasp

    #         # 向下移動以準備抓取物體
    #         grasp_demo.move_cartesian_byNow(0, 0 , 0.15) # avoid grasp other object by mistake
    #         # 夾爪抓取
    #         grasp_demo.get_it()
    #         # 向上移動
    #         grasp_demo.move_cartesian_byNow(0, 0 , -0.35)
    #         rospy.sleep(0.5)
    #         # 利用joint0製造平移路徑
    #         grasp_demo.move_by_joints(3.14159)

    #         grasp_demo.shutDown()

        

        # results_pub = rospy.Publisher('/arm_results', Bool, queue_size=10)
        # result_msg = Bool()
        # result_msg.data = waitForget
        # results_pub.publish(result_msg)