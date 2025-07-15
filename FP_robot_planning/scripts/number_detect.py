#!/usr/bin/env python3
# coding=utf-8

import rospy
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Bool
from cv_bridge import CvBridge
import cv2
import torch
from torch import nn
from torchvision import transforms
import numpy as np
import rospkg
from PIL import Image as PILImage

class Net(torch.nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1,16,kernel_size=(3,3))  # 第一卷積層(特徵提取)
        self.conv2 = nn.Conv2d(16,32,kernel_size=(3,3)) # 第二卷積層(特徵提取)
        self.maxpool = nn.MaxPool2d(kernel_size=(2,2))  # 最大池化層(加強特徵提取)
        self.lin1 = nn.Linear(800,128)
        self.out = nn.Linear(128,10) # 模型的最後一層為線性層，用來處理CNN扁平化後的輸出，總共輸出10類，數字0~9
    def forward(self,x):
        # (1, 28, 28)
        x = self.conv1(x) # (16, 26, 26)
        x = nn.functional.relu(x) # 選用ReLU為激活函數(16, 26, 26)
        x = self.maxpool(x) # (16, 13, 13)
        x = self.conv2(x) # (32, 11, 11)
        x = nn.functional.relu(x)# (32, 11, 11)
        x = self.maxpool(x) # (32, 5, 5)
        x = x.flatten(start_dim=1) # 扁平化(800)
        x = self.lin1(x) # (128)
        x = nn.functional.relu(x) # (128)
        x = self.out(x) # (10)
        x = nn.functional.log_softmax(x,dim=1) # 使用log_softmax( )，以機率的形式進行分類
        return x

class NumberRecognizerNode:
    def __init__(self):
        self.bridge = CvBridge()
        # 定義網路與device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = Net()
        # 尋找網路訓練後的參數，載入device後將設置成評估模式
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('myrobot_1100410_description')
        model_path = os.path.join(package_path, 'models/mnist_cnn.pt')
        self.model.load_state_dict(torch.load(model_path, weights_only=True))
        self.model.to(self.device)
        self.model.eval()

        # print("Model parameters loaded:", self.model.state_dict().keys())
        
        # 圖片轉換(用來輸入至網路)
        self.transform = transforms.Compose([
            # transforms.Resize((28, 28)),              # 擷取圖片時已調整成28*28
            transforms.ToTensor(),                      # 轉換成Tensor格式
            transforms.Normalize((0.1307,), (0.3081,))  # 正規化參數(官方提供)
        ])
        # 初始化節點
        rospy.init_node('number_detect', anonymous=True)
        # 訂閱/image_raw話題(辨識數字)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        # 訂閱/pos_results話題(判定是否要辨識數字)
        self.pos_sub = rospy.Subscriber("/pos_results", Bool, self.pos_callback)
        self.results_pub = rospy.Publisher('/number_results', Int32MultiArray, queue_size=10)
        self.fake_results_pub = rospy.Publisher('/fake_number_results', Int32MultiArray, queue_size=10)


    
    def process_image(self, cv_image):   
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # 將cv_image轉換成HSV格式
        # 綠色的HSV範圍
        # lower_green = np.array([50, 50, 50])    # 範圍下限
        # upper_green = np.array([80, 255, 255])  # 範圍上限  
        lower_green = np.array([30, 40, 40])
        upper_green = np.array([90, 255, 255])         
        mask = cv2.inRange(hsv_image, lower_green, upper_green)                          # 綠色區域的mask        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # 取得綠色區域的輪廓        
        split_images = [] # 用來儲存切割後的單一數字

        # 分別處理各個綠色區域的輪廓
        for contour in contours:            
            x, y, w, h = cv2.boundingRect(contour) # 取得綠色區域的輪廓的邊界值                       
            cropped_image = cv_image[y:y+h, x:x+w] # 裁剪圖片，並得到單一數字圖片
            gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)           # 將單一數字圖片轉成灰階(因網路使用黑白圖像訓練)            
            _, binary_image = cv2.threshold(gray_image, 1, 255, cv2.THRESH_BINARY) # 將灰階圖片轉成黑白圖片(大於1是白色，小於1則是黑色)
            # 以下99~114行用於將黑白圖片調整到目標大小，並保留長寬比
            original_height, original_width = binary_image.shape[:2] #取得黑白圖片長寬 
            target_size = 20  # 目標大小20*20
            # 計算長寬比
            if original_width > original_height:
                new_width = target_size
                new_height = int((target_size / original_width) * original_height)
            else:
                new_height = target_size
                new_width = int((target_size / original_height) * original_width)
            # 調整黑白圖片至20*20
            resized_image = cv2.resize(binary_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            canvas = np.zeros((target_size, target_size), dtype=np.uint8) # 創建20*20黑色canvas當背景
            # 將黑白圖片放置canvas中間
            top = (target_size - new_height) // 2
            left = (target_size - new_width) // 2
            canvas[top:top+new_height, left:left+new_width] = resized_image 
            # 以下116~122行用於將黑白圖片調整到網路輸入大小
            net_input_size = 28 # 網路輸入為28*28
            padded_image = np.zeros((net_input_size, net_input_size), dtype=np.uint8) # 創建28*28黑色canvas當背景
            # 將黑白圖片放置canvas中間
            x_offset = (net_input_size - target_size) // 2
            y_offset = (net_input_size - target_size) // 2
            padded_image[y_offset:y_offset+target_size, x_offset:x_offset+target_size] = canvas
            cv2.imshow(f'Cropped Image {contour}', padded_image)
            # 將numpy陣列轉換成PIL圖片
            pil_image = PILImage.fromarray(padded_image)
            split_images.append((x, pil_image))  # 儲存PIL圖片以及其x座標
        cv2.waitKey(0) 
        split_images.sort(key=lambda item: item[0])       # 根據x座標由左至右排序圖片
        split_images = [item[1] for item in split_images] # 只需要回傳圖片
        global process_image
        process_image = False
        return split_images if split_images else None

        
    def callback(self, data):
        global process_image
        global results

        if(process_image==True):        
            print(process_image)
            results = []
            # 處理/image_raw的圖片，轉換成bgr8 cv_image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # 得到切割後的PIL圖片，用來判斷多個數字`
            split_images = self.process_image(cv_image)
            # 判斷數字
            for _, pil_image in enumerate(split_images): # 依序處理PIL圖片
                image_tensor = self.transform(pil_image).unsqueeze(0).to(self.device) # 將PIL圖片轉換成網路輸入對應格式
                # pil_np = np.array(pil_image)       # 將PIL圖片轉換成numpy陣列，用來可視化
                # cv2.imshow(f"Digit {i+1}", pil_np) # 顯示PIL圖片

                # 網路推理
                with torch.no_grad():
                    output = self.model(image_tensor)
                    pred = torch.exp(output)        # 獲得每個類別的預測機率
                    pred_label = pred.argmax(dim=1) # 獲得機率最大值的索引值
                    results.append(pred_label.item())
            
            # 輸出最終結果
            rospy.loginfo(f"Predicted labels: {results}")
            # cv2.waitKey(1)

        # 發布topic(辨識到的數字)
        result_msg = Int32MultiArray()
        result_msg.data = results
        self.results_pub.publish(result_msg)
        # 發布假topic(讓number_nav節點的服務能持續執行)
        self.fake_results_pub.publish(result_msg)

    def pos_callback(self, data):
        global process_image
        # 如果機器人位於home
        if(data.data==True):
            # 將辨識數字(處理圖片)設成ture
            process_image = True
            
if __name__ == '__main__':
    process_image = True
    results = [] # 用來儲存判斷結果
    NumberRecognizerNode()
    rospy.spin()
