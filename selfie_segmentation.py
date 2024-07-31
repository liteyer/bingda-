#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy  # 导入ROS Python客户端库
import cv2  # 导入OpenCV库，用于图像处理
from cv_bridge import CvBridge, CvBridgeError  # 导入用于ROS和OpenCV之间图像消息转换的库
from sensor_msgs.msg import Image  # 导入ROS图像消息类型
import mediapipe as mp  # 导入Mediapipe库，用于自拍照分割
import numpy as np  # 导入NumPy库，用于数组操作

mp_drawing = mp.solutions.drawing_utils  # 导入Mediapipe的绘图工具
mp_selfie_segmentation = mp.solutions.selfie_segmentation  # 导入Mediapipe的SelfieSegmentation工具，用于自拍照分割

class MediaPipe:
    def __init__(self):    
        self.image_pub = rospy.Publisher("selfie_segmentation", Image, queue_size=1)  # 创建一个ROS发布者，用于发布自拍照分割结果
        self.bridge = CvBridge()  # 创建一个CvBridge实例，用于ROS图像消息和OpenCV图像之间的转换
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)  # 创建一个ROS订阅者，订阅USB摄像头的图像
        self.BG_COLOR = (192, 192, 192)  # gray
        self.cv_image = []  # 初始化一个空列表，用于存储最新的OpenCV图像
        self.new_image_flag = False  # 初始化一个标志，用于指示是否有新的图像需要处理

        while not rospy.is_shutdown():  # 在ROS节点运行期间持续循环
            with mp_selfie_segmentation.SelfieSegmentation(model_selection=1) as selfie_segmentation:  # 使用Mediapipe的SelfieSegmentation工具进行自拍照分割
                if not self.new_image_flag:  # 如果没有新的图像，则继续循环
                    continue
                else:
                    self.new_image_flag = False  # 重置新图像标志
                    image = self.cv_image  # 获取最新的OpenCV图像
                    bg_image = None
                    # Flip the image horizontally for a later selfie-view display, and convert
                    # the BGR image to RGB.
                    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                    # 为了提高性能，可以将图像标记为不可写，以便按引用传递
                    image.flags.writeable = False
                    results = selfie_segmentation.process(image)

                    image.flags.writeable = True  # 将图像标记为可写，以便进行绘图
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                    # 在背景图像上绘制自拍照分割
                    condition = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.1
                    # 背景可以自定义
                    if bg_image is None:
                        bg_image = np.zeros(image.shape, dtype=np.uint8)
                        bg_image[:] = self.BG_COLOR
                        output_image = np.where(condition, image, bg_image)

            # 将cv图像转换为ROS主题
            try:
                img_msg = self.bridge.cv2_to_imgmsg(output_image, "bgr8")  # 将OpenCV图像转换为ROS图像消息
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
        rospy.init_node("selfie_segmentation")  # 初始化ROS节点，节点名为"selfie_segmentation"
        rospy.loginfo("Starting selfie_segmentation node")  # 使用ROS日志系统打印节点启动信息
        MediaPipe()  # 创建MediaPipe类的实例
        rospy.spin()  # 保持节点运行，直到节点被关闭
    except KeyboardInterrupt:
        print ("Shutting down selfie_segmentation node.")  # 打印节点关闭信息
        cv2.destroyAllWindows()  # 关闭所有OpenCV创建的窗口
