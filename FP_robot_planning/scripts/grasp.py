#! /usr/bin/env python3
#coding=utf-8
import cv2
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from FP_robot_planning.srv import ArmGrasp, ArmGraspResponse 


class graspDemo:
    def __init__(self):
        # 初始化moveit控制器
        moveit_commander.roscpp_initialize(sys.argv)
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

    # 控制機械臂移動至指定 pose位置、orientation方向
    def move(self, pose, orientation):
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
        # rospy.sleep(1)

    @property
    def poseNow(self):
        # 返回當前座標
        return self.arm.get_current_pose(self.end_effector_link)
    
    # 笛卡爾規劃(從當前座標移動機械臂)
    def move_cartesian_byNow(self, x, y, z):    
        waypoints = []
        pos = self.poseNow.pose
        pos.position.x -= x
        pos.position.y -= y
        pos.position.z -= z
        waypoints.append(pos)
        fraction = 0.0 
        maxtries = 100
        attempts = 0
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路點列表
                                    0.1,         # eef_step，終端步進值
                                    0.0,         # jump_threshold，跳躍閾值
                                    True)        # 避障規劃
            attempts += 1
            
        if fraction == 1.0:
            self.arm.execute(plan)
            return True
        else:
            pass
        return False
    
    # 移動至命名的pose(已定義的姿態，如:home, scan, put)    
    def go_named(self, place):
        self.arm.set_named_target(place)
        self.arm.go(wait=True)
        
    #夾爪夾取
    def get_it(self):
        self.gripper.set_joint_value_target([0.16, -0.16])
        self.gripper.go()
        print("已抓取")
        # rospy.sleep(1)

    #夾爪放開
    def put_it(self):
        global findObject
        self.gripper.set_joint_value_target([0, 0])
        self.gripper.go()
        findObject = False
        # rospy.sleep(1)

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
    return [x, -y, z, 1]    #相機與圖片坐標系上下左右正負相反

# 獲取base_frame到reference_frame的旋轉平移矩陣，通過tf變換獲取
def get_RT_matrix(base_frame, reference_frame):
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

# 相機系轉base_link座標系，先旋轉再平移
def coordinate_transform(cameraFrame_pos, R, T):  
    worldFrame_pos = R.I * np.mat(cameraFrame_pos).T 
    worldFrame_pos[0,0] = worldFrame_pos[0,0] + T[0]
    worldFrame_pos[1,0] = worldFrame_pos[1,0] + T[1]
    worldFrame_pos[2,0] = worldFrame_pos[2,0] + T[2]
    worldFrame_pos = [worldFrame_pos[0,0], worldFrame_pos[1,0], worldFrame_pos[2,0]]
    # print(worldFrame_pos)
    return worldFrame_pos

# 獲取棧板放置位置(1,2,3)
def num_callback(msg):
    global pos, put_pos
    if pos == "home":
        last_two = int(f"{msg.data[2]}{msg.data[3]}")
        put_pos = str(last_two)

def getBoxCallBack(data):
    global findObject, u, v, put_pos, pos
    if pos == "home":
        # 將ROS圖片訊息轉換成OpenCV圖片
        bridge = CvBridge()
        # 處理/image_raw的圖片，轉換成bgr8格式的OpenCV圖片
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        # 將BGR圖片轉換為HSV色彩空間
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 定義橘色的HSV範圍(下界和上界)
        lower_orange = np.array([10, 100, 100]) # 橘色的下界 
        upper_orange = np.array([25, 255, 255]) # 橘色的上界
        # 獲取遮罩(保留橘色的部分)
        mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
        # 找到遮罩中所有的輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 找到最大的輪廓(假設最大的輪廓是目標箱子)
            largest_contour = max(contours, key=cv2.contourArea)
            # 獲取輪廓的中心
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                u = int(M["m10"] / M["m00"]) # 計算中心的u座標(水平)
                v = int(M["m01"] / M["m00"]) # 計算中心的v座標(垂直)
                # 在圖片中繪製中心點(綠色圓點)，並將findObject設為True
                cv2.circle(cv_image, (u, v), 5, (0, 255, 0), -1) 
                findObject = True

        cv2.imshow('Detected Box', cv_image)
        cv2.waitKey(1)

    else:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 定義紅色的HSV範圍(下界和上界)
        lower_red1 = np.array([0, 100, 100])    # 紅色的第一段下界
        upper_red1 = np.array([10, 255, 255])   # 紅色的第一段上界
        lower_red2 = np.array([160, 100, 100])  # 紅色的第二段下界
        upper_red2 = np.array([180, 255, 255])  # 紅色的第二段上界
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1) # 紅色第一段遮罩
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2) # 紅色第二段遮罩  
        mask_red = cv2.bitwise_or(mask_red1, mask_red2) # 合併遮罩  

        contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            # 計算每個區段的寬度(將矩形水平三等分)
            segment_width = w // 3
            # 計算每個區段中心點的座標
            u3, v3 = x + segment_width // 2, y + h // 2  # 第三段中心
            u2, v2 = x + segment_width + segment_width // 2, y + h // 2  # 第二段中心
            u1, v1 = x + 2 * segment_width + segment_width // 2, y + h // 2  # 第一段中心
            # 根據要放置在棧板上的位置(1,2,3)，給予對應的放置點
            if put_pos=='1':
                u, v = u1, v1

            elif put_pos=='2':
                u, v = u2, v2
                
            elif put_pos=='3':
                u, v = u3, v3
            print("put_pos:",put_pos, "u,v:",(u1, v1),(u2, v2),(u3, v3),(u, v))
            # 繪製三等分點
            cv2.circle(cv_image, (u1, v1), 5, (0, 255, 0), -1)
            cv2.circle(cv_image, (u2, v2), 5, (255, 255, 0), -1)
            cv2.circle(cv_image, (u3, v3), 5, (255, 0, 255), -1)

        cv2.imshow('Detected Box', cv_image)
        cv2.waitKey(1)

# 深度圖像回調函數
def depthCallback(data):
    global depthImg,depthOK
    depthImg = CvBridge().imgmsg_to_cv2(data, data.encoding)
    # print(depthImg[v,u])
    depthOK = True

#service服務
def grasp_service(req):
    global u, v, pos
    global depthOK, findObject
    # 在home時
    if req.pos == "home":
        rospy.loginfo('開始掃描')
        # 機械臂移動到scan位置
        grasp_demo.go_named('scan') # 移動到可以找到目標的位置
        rospy.loginfo('結束掃描')
        # 如果檢測到待抓取物體且獲取到了深度圖
        if depthOK and findObject:
            # 獲取相機系到base_link的旋轉平移矩陣
            [R, T] = get_RT_matrix('base_link', 'camera2_1')           
            # print(u, v, depthImg[v,u])
            # 圖片座標轉相機系座標
            cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u])
            # 相機系座標轉base_link座標
            worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
            print(' pose is: ', worldFrame_pos)            
            # 移動到抓取物體的上方
            grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.25],endDown)
            # 向下移動以準備抓取物體
            grasp_demo.move_cartesian_byNow(0, 0 , 0.13)
            # 夾爪抓取
            grasp_demo.get_it()
            # 向上移動
            grasp_demo.move_cartesian_byNow(0, 0 , -0.35)
            # 機械臂移動到put位置
            grasp_demo.go_named('put')
            result = "準備歸貨"
            pos = "put" 
            #回傳服務訊息("put")給number_nav節點
            return ArmGraspResponse(result)
    #在棧板位置時
    else:
        grasp_demo.go_named('scan')
        rospy.loginfo('開始掃描')
        # 機械臂移動到scan位置
        grasp_demo.go_named('scan') # 移動到可以找到目標的位置
        rospy.loginfo('結束掃描')
        # 獲取相機系到base_link的旋轉平移矩陣
        [R, T] = get_RT_matrix('base_link', 'camera2_1')
        u, v = int(u), int(v)  # 防止檢測的座標越界
        # print(u, v, depthImg[v,u])
        # 圖像座標轉相機系座標
        cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u])
        # 相機系座標轉base_link座標
        worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
        print(' pose is: ', worldFrame_pos)
        # 移動到放置在棧板上位置的上方
        grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.4],endDown)
        # 夾爪放開
        grasp_demo.put_it()
        # 機械臂回到待機位置
        grasp_demo.go_named('home') 
        pos = "home" 
        #回傳服務訊息("home")給number_nav節點
        return ArmGraspResponse("準備回歸")

# 向下對應的 xyzw 座標
endDown = [1, 0, 0, 0]
# endDown = [0.000060, 0.999987, 0.000000, 0.005107]
# endDown = [0.9997, 0.000026, 0.0011835, 0.0000018]
# endDown = [0.99982, -0.00004, 0.0168, 0.000001]

if __name__ == "__main__":
    # 初始化 ROS 節點
    rospy.init_node('grasp')
    # 深度相機圖片對應的topic名稱
    depth_topic = '/camera/depth/image_raw'
    color_topic = '/camera/color/image_raw'
    # 實例化抓取控制類
    grasp_demo = graspDemo()
    # 機械臂移動到待機位置
    grasp_demo.go_named('home') 
    # 初始化各參數
    u = 320
    v = 240
    pos = "home"
    put_pos=""
    findObject = False
    depthOK = False
    # 訂閱各個topic  
    rospy.Subscriber(depth_topic, Image, depthCallback, queue_size=1)
    rospy.Subscriber(color_topic, Image, getBoxCallBack, queue_size=1)
    rospy.Subscriber("/number_results", Int32MultiArray, num_callback, queue_size=1)
    # 服務連接
    rospy.Service('arm_grasp', ArmGrasp, grasp_service)
    rospy.spin()
