#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import os
import time

def monitor_txt_file(file_path, topic_name, check_interval=1.0):
    # 初始化ROS节点
    rospy.init_node('duqu', anonymous=True)
    # 创建一个发布者
    pub = rospy.Publisher(topic_name, String, queue_size=10)

    last_mod_time = 0
    while not rospy.is_shutdown():
        # 获取当前文件的修改时间
        current_mod_time = os.path.getmtime(file_path)
        
        # 如果文件的修改时间发生了变化
        if current_mod_time != last_mod_time:
            # 读取文件内容
            with open(file_path, 'r') as file:
                content = file.read()
                
            # 发布文件内容
            content = String()
            pub.publish(content)
            #rospy.loginfo(f"File {file_path} modified, content published.")
            rospy.loginfo("%s",content)
            # 更新最后修改时间
            last_mod_time = current_mod_time

        # 等待一段时间再次检查
        time.sleep(check_interval)

if __name__ == '__main__':
    # 假设你的txt文件位于ROS包的同一目录下，名为'data.txt'
    # 并且你想要发布到名为'txt_data'的话题上
    file_path = rospy.get_package_path('/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts') + '/yolosign.txt'
    topic_name = 'txt_data'
    
    # 运行文件监视器
    monitor_txt_file(file_path, topic_name)
    
    
    
    
    
    

'''#!/usr/bin/env python  
# -*- coding: utf-8 -*-  
import rospy  
from std_msgs.msg import String  
  
def callback(data):  
    # 这里的data就是std_msgs/String的data字段，即你接收到的字符串  
    rospy.loginfo("I heard: %s", data.data)  
  
    # 根据消息内容执行不同的操作  
    if "data1" in data.data:  
        # 执行与data1相关的操作  
        print("Executing operation for data1")  
        # 你可以在这里设置其他变量或发布其他消息  
  
    elif "data2" in data.data:  
        # 执行与data2相关的操作  
        print("Executing operation for data2")  
        # ...  
  
    # 以此类推，你可以添加更多的条件判断  
  
def listener():  
    # 初始化节点  
    rospy.init_node('listener', anonymous=True)  
  
    # 创建一个订阅者，订阅名为'chatter'的topic，消息类型为std_msgs/String，回调函数为callback  
    rospy.Subscriber("chatter", String, callback)  
  
    # 保持节点运行，等待回调  
    rospy.spin()  
  
if __name__ == '__main__':  
    listener()
'''