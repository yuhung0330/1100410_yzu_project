#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage

def callback(data):
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
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
            # 繪製左右兩側的把手點
            handle_offset = 100  # 假設把手從中心點左右各延伸50個像素

            # 左邊把手點
            cv2.circle(cv_image, (cx - handle_offset, cy), 5, (0, 255, 0), -1)

            # 右邊把手點
            cv2.circle(cv_image, (cx + handle_offset, cy), 5, (0, 255, 0), -1)

        cv2.drawContours(cv_image, [largest_contour], -1, (255, 0, 0), 2)

    cv2.imshow('Detected Box', cv_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    # 初始化節點
    rospy.init_node('move_it_get', anonymous=True)
    # 訂閱/image_raw話題
    image_sub = rospy.Subscriber("/image_raw", Image, callback)
    rospy.spin()