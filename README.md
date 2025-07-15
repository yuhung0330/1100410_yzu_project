# 使用說明

ira_laser_tools請使用 https://github.com/iralabdisco/ira_laser_tools

將 num_pkg 內的檔案移至：usr\share\gazebo-11\media\materials\textures

將 material_pkg 內的檔案移至：usr\share\gazebo-11\media\materials\scripts

分別執行以下指令：
roslaunch FP_robot_gazebo FP_robot_nav.launch
roslaunch FP_robot_nav nav.launch
rosrun FP_robot_planning grasp.py
rosrun FP_robot_planning number_nav.py
rosrun FP_robot_planning number_detect.py


