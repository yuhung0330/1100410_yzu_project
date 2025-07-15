#!/usr/bin/env python3
# coding=utf-8
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random

bridge = CvBridge()
color_detected = 0
random_detected = False

def image_callback(msg):
    global color_detected, random_detected
    if color_detected == 0 and random_detected == False:
        try:
            # 將ROS圖像消息轉換為OpenCV格式
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return
        # 執行顏色檢測
        detected_color = detect_color(cv_image)
        rospy.loginfo("辨識到的顏色: %s", detected_color)
        navigate_to_waypoint(detected_color)
        # 設定顏色檢測標誌為1，表示已經檢測過一次顏色
        color_detected = 1
    if color_detected == 2:
        randomNav()

# 假設的顏色檢測函數
def detect_color(cv_image):
    # 將BGR圖像轉換為HSV圖像
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # 定義HSV顏色的範圍
    lower_blue = np.array([100, 50, 50])  # 藍色的HSV下界
    upper_blue = np.array([130, 255, 255])  # 藍色的HSV上界
    lower_red1 = np.array([0, 50, 50])  # 紅色的HSV下界1
    upper_red1 = np.array([10, 255, 255])  # 紅色的HSV上界1
    lower_red2 = np.array([170, 50, 50])  # 紅色的HSV下界2
    upper_red2 = np.array([180, 255, 255])  # 紅色的HSV上界2
    lower_green = np.array([50, 50, 50])  # 綠色的HSV下界
    upper_green = np.array([80, 255, 255])  # 綠色的HSV上界
    lower_black = np.array([0, 0, 0])  # 黑色的HSV下界
    upper_black = np.array([180, 255, 30])  # 黑色的HSV上界
    # 根據顏色範圍創建掩碼
    mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
    mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask_red = mask_red1 + mask_red2  # 將兩個紅色掩碼合併
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_image, lower_black, upper_black)
    # 檢測顏色
    if np.any(mask_blue):
        return 'blue'
    elif np.any(mask_red):
        return 'red'
    elif np.any(mask_green):
        return 'green'
    elif np.any(mask_black):
        return 'black'
    else:
        return 'unknown'

# 導航結果回調函數
def navigate_to_waypoint(color):
    # 根據顏色選擇導航的航點
    if color == 'blue':
        waypoint = '1'
    elif color == 'red':
        waypoint = '2'
    elif color == 'green':
        waypoint = '3'
    elif color == 'black':
        waypoint = '4'
    else:
        rospy.logwarn("未能檢測到有效的顏色!默認導航到點3。")
        waypoint = '3'

    # 構建航點名稱消息包
    msg = String()
    msg.data = waypoint
    # 發送航點名稱消息包
    navi_pub.publish(msg)

def randomNav():
    global color_detected, random_detected
    waypoint = str(random.randint(5, 8))
    # 構建航點名稱消息包
    msg2 = String()
    msg2.data = waypoint
    # 發送航點名稱消息包
    navi_pub.publish(msg2)
    rospy.loginfo(f"導航到隨機航點進行顏色辨識: {waypoint}")
    color_detected = 0
    

def resultNavi(done):
    global color_detected, random_detected
    rospy.loginfo("導航結果 = %s", done.data)
    if done.data == "done" and color_detected == 1:
        color_detected = 2
        random_detected = True
    if done.data == "done" and color_detected == 0:
        random_detected = False
    
if __name__ == "__main__":
    rospy.init_node("fpNav")
    # 發佈航點名稱話題
    navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10)
    result_sub = rospy.Subscriber("/waterplus/navi_result", String, resultNavi, queue_size=10)
    # 延時1秒鐘，讓後台的話題發佈操作能夠完成
    image_sub = rospy.Subscriber("/image_raw", Image, image_callback, queue_size=1)
    rospy.sleep(1)
    rospy.spin()
