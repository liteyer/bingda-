#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 手势分类部分参考博客:https://blog.csdn.net/weixin_45930948/article/details/115444916
# 导入ROS相关的库  
import rospy  
# 导入OpenCV库，用于图像处理  
import cv2  
# 导入cv_bridge库，用于在ROS和OpenCV之间转换图像格式  
from cv_bridge import CvBridge, CvBridgeError  
# 导入ROS中的图像消息类型  
from sensor_msgs.msg import Image  
# 导入ROS中的字符串消息类型  
from std_msgs.msg import String  
  
# 导入MediaPipe库及其相关模块，用于手势识别  
import mediapipe as mp  
import math  
mp_drawing = mp.solutions.drawing_utils  
mp_drawing_styles = mp.solutions.drawing_styles  
mp_hands = mp.solutions.hands  
  
# 定义一个图像转换器类  
class image_converter:  
    def __init__(self):  
        # 初始化一个发布器，用于发布处理后的图像  
        self.image_pub = rospy.Publisher("hands", Image, queue_size=1)  
        # 初始化一个发布器，用于发布手势识别的信息  
        self.info_pub = rospy.Publisher("hands_info", String, queue_size=1)  
        # 创建一个cv_bridge对象，用于图像格式的转换  
        self.bridge = CvBridge()  
        # 创建一个订阅器，用于从/usb_cam/image_raw话题订阅原始图像  
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)  
        # 初始化一个空列表，用于存储图像数据  
        self.cv_image = []  
        # 初始化一个字符串消息对象  
        self.message = String()  
        # 初始化一个新图像标志  
        self.new_image_flag = False  
  
        # 持续运行，直到ROS节点被关闭  
        while not rospy.is_shutdown():  
            # 使用MediaPipe的手部解决方案创建一个手部识别对象  
            with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:  
                # 如果没有新图像，则继续等待  
                if not self.new_image_flag:  
                    continue  
                else:  
                    # 如果有新图像，则重置新图像标志  
                    self.new_image_flag = False  
                    # 获取图像数据  
                    image = self.cv_image  
                    # 将图像从BGR格式转换为RGB格式  
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  
                    # 使用MediaPipe处理图像，获取手部识别的结果  
                    results = hands.process(image)  
  
                    # 将图像从RGB格式转换回BGR格式，并准备进行绘制  
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  
                    # 如果识别到多个手的手势  
                    if results.multi_hand_landmarks:  
                        gesture_counter = 1  
                        # 遍历每个手的手势  
                        for hand_landmarks in results.multi_hand_landmarks:  
                            # 在图像上绘制手部关键点和连接线  
                            mp_drawing.draw_landmarks(  
                                image,  
                                hand_landmarks,  
                                mp_hands.HAND_CONNECTIONS,  
                                mp_drawing_styles.get_default_hand_landmarks_style(),  
                                mp_drawing_styles.get_default_hand_connections_style())  
                            # 获取手部关键点的本地坐标  
                            hand_local = []  
                            for i in range(21):  
                                x = hand_landmarks.landmark[i].x * image.shape[1]  
                                y = hand_landmarks.landmark[i].y * image.shape[0]  
                                hand_local.append((x, y))  
                            # 如果有手部关键点数据  
                            if hand_local:  
                                # 计算手部关键点之间的角度  
                                angle_list = self.hand_angle(hand_local)  
                                # 根据角度判断手势  
                                gesture_str = self.h_gesture(angle_list)  
                                # 如果识别到手势  
                                if gesture_str:  
                                    # 设置手势信息  
                                    self.message.data = gesture_str  
                                # 在图像上显示手势信息  
                                cv2.putText(image, gesture_str, (200 * gesture_counter, 50), 0, 2.0, (0, 0, 255), 5)  
                                gesture_counter += 1  
  
                # 尝试将处理后的图像转换为ROS消息格式，并发布  
                try:  
                    img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")  
                    img_msg.header.stamp = rospy.Time.now()  
                    # 发布手势信息  
                    self.info_pub.publish(self.message)  
                    # 重置手势信息  
                    self.message.data = "None"  
                    # 发布处理后的图像  
                    self.image_pub.publish(img_msg)  
                # 如果转换过程中出现错误，则打印错误信息  
                except CvBridgeError as e:  
                    print(e)  



    # 这个方法是用来从一个ROS话题转换为一个OpenCV图像  
    def callback(self, data):  
        try:  
            # 使用bridge将ROS的图像消息转换为OpenCV的图像格式  
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  
            # 设置新图像标志为True  
            self.new_image_flag = True  
        except CvBridgeError as e:  
            # 如果转换过程中发生错误，打印错误信息  
            print(e)  

    # 这个方法用来计算两个二维向量之间的角度
    def vector_2d_angle(self, v1, v2):  
        # 分别提取两个向量的x和y分量  
        v1_x, v1_y = v1[0], v1[1]  
        v2_x, v2_y = v2[0], v2[1]  
        try:  
            # 使用反余弦函数计算两个向量的角度，并将结果从弧度转换为角度  
            angle_ = math.degrees(math.acos((v1_x * v2_x + v1_y * v2_y) / ((v1_x ** 2 + v1_y ** 2) ** 0.5 * (v2_x ** 2 + v2_y ** 2) ** 0.5)))  
        except:  
            # 如果计算过程中发生错误，将角度设置为一个特殊值  
            angle_ = 65535.  
        # 如果计算出的角度大于180度，也将其设置为特殊值  
        if angle_ > 180.:  
            angle_ = 65535.  
        return angle_  
    
    def hand_angle(self,hand_):
        '''
            获取对应手相关向量的二维角度,根据角度确定手势
        '''
        # 对于大拇指，食指，中指，无名指和小拇指，分别计算其与手掌中心点的角度，并添加到列表中 
        angle_list = []
        #---------------------------- thumb 大拇指角度
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0])- int(hand_[2][0])),(int(hand_[0][1])-int(hand_[2][1]))),
            ((int(hand_[3][0])- int(hand_[4][0])),(int(hand_[3][1])- int(hand_[4][1])))
            )
        angle_list.append(angle_)
        #---------------------------- index 食指角度
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0])-int(hand_[6][0])),(int(hand_[0][1])- int(hand_[6][1]))),
            ((int(hand_[7][0])- int(hand_[8][0])),(int(hand_[7][1])- int(hand_[8][1])))
            )
        angle_list.append(angle_)
        #---------------------------- middle 中指角度
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0])- int(hand_[10][0])),(int(hand_[0][1])- int(hand_[10][1]))),
            ((int(hand_[11][0])- int(hand_[12][0])),(int(hand_[11][1])- int(hand_[12][1])))
            )
        angle_list.append(angle_)
        #---------------------------- ring 无名指角度
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0])- int(hand_[14][0])),(int(hand_[0][1])- int(hand_[14][1]))),
            ((int(hand_[15][0])- int(hand_[16][0])),(int(hand_[15][1])- int(hand_[16][1])))
            )
        angle_list.append(angle_)
        #---------------------------- pink 小拇指角度
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0])- int(hand_[18][0])),(int(hand_[0][1])- int(hand_[18][1]))),
            ((int(hand_[19][0])- int(hand_[20][0])),(int(hand_[19][1])- int(hand_[20][1])))
            )
        angle_list.append(angle_)
        return angle_list

    def h_gesture(self,angle_list):
        '''
            # 二维约束的方法定义手势
            # fist five gun love one six three thumbup yeah
        '''
        # 定义两个阈值用于区分弯曲和伸直的手指
        thr_angle_curve = 49.
        thr_angle_straighten = 65.0
        gesture_str = None
        # 检查角度列表中是否存在特殊值
        if 65535. not in angle_list:
            # 根据不同的角度组合确定手势  
            if (angle_list[0]>thr_angle_straighten)  and (angle_list[1]<thr_angle_curve) and (angle_list[2]>thr_angle_straighten) and (angle_list[3]>thr_angle_straighten) and (angle_list[4]>thr_angle_straighten):
                gesture_str = "one"
            elif (angle_list[0]>thr_angle_straighten) and (angle_list[1]<thr_angle_curve) and (angle_list[2]<thr_angle_curve) and (angle_list[3]>thr_angle_straighten) and (angle_list[4]>thr_angle_straighten):
                gesture_str = "two"   
            elif (angle_list[0]>thr_angle_straighten) and (angle_list[1]<thr_angle_curve) and (angle_list[2]<thr_angle_curve) and (angle_list[3]<thr_angle_curve) and (angle_list[4]>thr_angle_straighten):
                gesture_str = "three"      
            elif (angle_list[0]>thr_angle_straighten) and (angle_list[1]>thr_angle_straighten) and (angle_list[2]<thr_angle_curve) and (angle_list[3]<thr_angle_curve) and (angle_list[4]<thr_angle_curve):
                gesture_str = "three"     
            elif (angle_list[0]>thr_angle_straighten) and (angle_list[1]<thr_angle_curve) and (angle_list[2]<thr_angle_curve) and (angle_list[3]<thr_angle_curve) and (angle_list[4]<thr_angle_curve):
                gesture_str = "four"                           
            elif (angle_list[0]<thr_angle_curve) and (angle_list[1]<thr_angle_curve) and (angle_list[2]<thr_angle_curve) and (angle_list[3]<thr_angle_curve) and (angle_list[4]<thr_angle_curve):
                gesture_str = "five"
            elif (angle_list[0]<thr_angle_curve)  and (angle_list[1]>thr_angle_straighten) and (angle_list[2]>thr_angle_straighten) and (angle_list[3]>thr_angle_straighten) and (angle_list[4]<thr_angle_curve):
                gesture_str = "six"
            elif (angle_list[0]<thr_angle_curve)  and (angle_list[1]>thr_angle_straighten) and (angle_list[2]>thr_angle_straighten) and (angle_list[3]>thr_angle_straighten) and (angle_list[4]>thr_angle_straighten):
                gesture_str = "Good"
            elif (angle_list[0]>thr_angle_straighten) and (angle_list[1]>thr_angle_straighten) and (angle_list[2]<thr_angle_curve) and (angle_list[3]>thr_angle_straighten) and (angle_list[4]>thr_angle_straighten):
                gesture_str = "F**K"
            # 如果都不符合，将手势设置为"None"  
            else:  
                gesture_str = "None"  
        return gesture_str


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("pose")
        rospy.loginfo("Starting pose node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down pose node.")
        cv2.destroyAllWindows()
