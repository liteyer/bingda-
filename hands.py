#!/usr/bin/env python
# -*- coding: utf-8 -*-

from email.mime import image  # 这行代码似乎是错误的，因为在这个脚本中并不需要导入这个模块
import rospy  # 导入ROS Python客户端库
import cv2  # 导入OpenCV库，用于图像处理
from cv_bridge import CvBridge, CvBridgeError  # 导入用于ROS和OpenCV之间图像消息转换的库
from sensor_msgs.msg import Image  # 导入ROS图像消息类型
import mediapipe as mp  # 导入Mediapipe库，用于手部姿势检测
import math  # 导入数学库，但在这个脚本中并未使用

mp_drawing = mp.solutions.drawing_utils  # 导入Mediapipe的绘图工具
mp_drawing_styles = mp.solutions.drawing_styles  # 导入Mediapipe的绘图样式
mp_hands = mp.solutions.hands  # 导入Mediapipe的手部姿势检测模型

class image_converter:
    def __init__(self):    
        self.image_pub = rospy.Publisher("hands", Image, queue_size=1)  # 创建一个ROS发布者，用于发布手部姿势检测结果
        self.bridge = CvBridge()  # 创建一个CvBridge实例，用于ROS图像消息和OpenCV图像之间的转换
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)  # 创建一个ROS订阅者，订阅USB摄像头的图像
        self.cv_image = []  # 初始化一个空列表，用于存储最新的OpenCV图像
        self.new_image_flag = False  # 初始化一个标志，用于指示是否有新的图像需要处理

        while not rospy.is_shutdown():  # 在ROS节点运行期间持续循环
            with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:  # 使用Mediapipe的手部姿势检测模型
                if not self.new_image_flag:  # 如果没有新的图像，则继续循环
                    continue
                else:
                    self.new_image_flag = False  # 重置新图像标志
                    image = self.cv_image  # 获取最新的OpenCV图像
                    # 为了提高性能，可以将图像标记为不可写，以便按引用传递
                    image.flags.writeable = False
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # 将BGR图像转换为RGB图像，以便Mediapipe处理
                    results = hands.process(image)  # 使用Mediapipe进行手部姿势检测

                    # 在图像上绘制手部姿势注释
                    image.flags.writeable = True  # 将图像标记为可写，以便进行绘图
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # 将RGB图像转换回BGR图像，以便OpenCV处理
                    if results.multi_hand_landmarks:  # 如果检测到手部姿势
                        for hand_landmarks in results.multi_hand_landmarks:  # 遍历所有检测结果
                            mp_drawing.draw_landmarks(  # 绘制手部姿势的标记
                                image,  # 图像
                                hand_landmarks,  # 手部姿势标记
                                mp_hands.HAND_CONNECTIONS,  # 手部标记连接
                                mp_drawing_styles.get_default_hand_landmarks_style(),  # 标记样式
                                mp_drawing_styles.get_default_hand_connections_style())  # 连接样式
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")  # 将OpenCV图像转换为ROS图像消息
                    img_msg.header.stamp = rospy.Time.now()  # 设置ROS图像消息的时间戳
                    self.image_pub.publish(img_msg)  # 发布ROS图像消息
                except CvBridgeError as e:
                    print (e)  # 打印转换错误

    def callback(self, data):
        # 将ROS主题转换为cv图像
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # 将ROS图像消息转换为OpenCV图像
            self.new_image_flag = True  # 设置新图像标志，指示有新的图像需要处理
        except CvBridgeError as e:
            print (e)  # 打印转换错误

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("pose")  # 初始化ROS节点，节点名为"pose"
        rospy.loginfo("Starting pose node")  # 使用ROS日志系统打印节点启动信息
        image_converter()  # 创建image_converter类的实例
        rospy.spin()  # 保持节点运行，直到节点被关闭
    except KeyboardInterrupt:
        print ("Shutting down pose node.")  # 打印节点关闭信息
        cv2.destroyAllWindows()  # 关闭所有OpenCV创建的窗口
