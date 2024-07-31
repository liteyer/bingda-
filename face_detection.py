#!/usr/bin/env python
# -*- coding: utf-8 -*-

from email.mime import image  # 这行应该是错误的，不应该在此处导入image模块
import rospy  # 导入ROS Python客户端库
import cv2  # 导入OpenCV库，用于图像处理
from cv_bridge import CvBridge, CvBridgeError  # 导入ROS和OpenCV之间的桥梁库
from sensor_msgs.msg import Image  # 导入ROS图像消息类型
import mediapipe as mp  # 导入Mediapipe库，用于人脸检测
import math  # 导入数学库，但在此代码中并未使用

mp_drawing = mp.solutions.drawing_utils  # 导入Mediapipe的绘图工具
mp_drawing_styles = mp.solutions.drawing_styles  # 导入Mediapipe的绘图样式
mp_face_detection = mp.solutions.face_detection  # 导入Mediapipe的人脸检测模型

class image_converter:
    def __init__(self):    
        self.image_pub = rospy.Publisher("face_detection", Image, queue_size=1)  # 创建一个ROS发布者，用于发布人脸检测结果
        self.bridge = CvBridge()  # 创建一个CvBridge实例，用于ROS图像消息和OpenCV图像之间的转换
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)  # 创建一个ROS订阅者，订阅USB摄像头图像
        self.cv_image = []  # 初始化一个空列表，用于存储OpenCV图像
        self.new_image_flag = False  # 初始化一个标志，用于指示是否有新的图像需要处理

        while not rospy.is_shutdown():  # 在ROS节点运行期间持续循环
            with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face_detection:  # 使用Mediapipe的人脸检测模型
                if not self.new_image_flag:  # 如果没有新的图像，则继续循环
                    continue
                else:
                    self.new_image_flag = False  # 重置新图像标志
                    image = self.cv_image  # 获取最新的OpenCV图像
                # 为了提高性能，可以将图像标记为不可写，以便按引用传递
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # 将BGR图像转换为RGB图像，以便Mediapipe处理
                results = face_detection.process(image)  # 使用Mediapipe进行人脸检测

                # 在图像上绘制人脸检测注释
                image.flags.writeable = True  # 将图像标记为可写，以便进行绘图
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # 将RGB图像转换回BGR图像，以便OpenCV处理
                if results.detections:  # 如果检测到人脸
                    for detection in results.detections:  # 遍历所有检测结果
                        mp_drawing.draw_detection(image, detection)  # 在图像上绘制检测结果
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")  # 将OpenCV图像转换为ROS图像消息
                    img_msg.header.stamp = rospy.Time.now()  # 设置ROS图像消息的时间戳
                    self.image_pub.publish(img_msg)  # 发布ROS图像消息
                except CvBridgeError as e:
                    print (e)  # 打印转换错误

    def callback(self,data):
        # 将ROS主题转换为cv图像
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # 将ROS图像消息转换为OpenCV图像
            self.new_image_flag = True  # 设置新图像标志，指示有新的图像需要处理
        except CvBridgeError as e:
            print (e)  # 打印转换错误

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("face_detection")  # 初始化ROS节点
        rospy.loginfo("Starting face_detection node")  # 打印节点启动信息
        image_converter()  # 创建image_converter实例
        rospy.spin()  # 保持节点运行，直到节点关闭
    except KeyboardInterrupt:
        print ("Shutting down face_detection node.")  # 打印节点关闭信息
        cv2.destroyAllWindows()  # 关闭所有OpenCV窗口
