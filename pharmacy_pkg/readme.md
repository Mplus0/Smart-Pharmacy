# 智慧药房竞赛程序启动指南

## 一、初始化导航
    roslaunch pharmacy_pkg smart_pharmacy_demo_init.launch
## 二、开启相机
    roslaunch astra_camera astra.launch
## 三、开启web视频服务
    rosrun web_video_server web_video_server
## 四、运行二维码识别程序
    cd ~/robot_ws/src/pharmacy_pkg/scripts/
    python detect_code_for_EPRobot.py
## 五、药房任务处理
    cd ~/robot_ws/src/pharmacy_pkg/scripts/
    python F1_yaofang.py