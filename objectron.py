#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy  # 导入ROS Python客户端库
import cv2  # 导入OpenCV库，用于图像处理
from cv_bridge import CvBridge, CvBridgeError  # 导入用于ROS和OpenCV之间图像消息转换的库
from sensor_msgs.msg import Image  # 导入ROS图像消息类型
import mediapipe as mp  # 导入Mediapipe库，用于物体检测

mp_drawing = mp.solutions.drawing_utils  # 导入Mediapipe的绘图工具
mp_objectron = mp.solutions.objectron  # 导入Mediapipe的Objectron工具，用于物体检测

class image_converter:
    def __init__(self):    
        self.image_pub = rospy.Publisher("hands", Image, queue_size=1)  # 创建一个ROS发布者，用于发布物体检测结果
        self.bridge = CvBridge()  # 创建一个CvBridge实例，用于ROS图像消息和OpenCV图像之间的转换
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)  # 创建一个ROS订阅者，订阅USB摄像头的图像
        self.cv_image = []  # 初始化一个空列表，用于存储最新的OpenCV图像
        self.new_image_flag = False  # 初始化一个标志，用于指示是否有新的图像需要处理

        while not rospy.is_shutdown():  # 在ROS节点运行期间持续循环
            if not self.new_image_flag:  # 如果没有新的图像，则继续循环
                continue
            else:
                self.new_image_flag = False  # 重置新图像标志
                image = self.cv_image  # 获取最新的OpenCV图像
            with mp_objectron.Objectron(static_image_mode=False,
                                    max_num_objects=5,
                                    min_detection_confidence=0.5,
                                    min_tracking_confidence=0.99,
                                    model_name='Shoe') as objectron:  # 使用Mediapipe的Objectron工具进行物体检测
                # 为了提高性能，可以将图像标记为不可写，以便按引用传递
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = objectron.process(image)

                # 在图像上绘制物体的框和轴
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.detected_objects:
                    for detected_object in results.detected_objects:
                        mp_drawing.draw_landmarks(
                            image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
                        mp_drawing.draw_axis(image, detected_object.rotation, detected_object.translation)
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
        rospy.init_node("hands")  # 初始化ROS节点，节点名为"hands"
        rospy.loginfo("Starting hands node")  # 使用ROS日志系统打印节点启动信息
        image_converter()  # 创建image_converter类的实例
        rospy.spin()  # 保持节点运行，直到节点被关闭
    except KeyboardInterrupt:
        print ("Shutting down hands node.")  # 打印节点关闭信息
        cv2.destroyAllWindows()  # 关闭所有OpenCV创建的窗口
