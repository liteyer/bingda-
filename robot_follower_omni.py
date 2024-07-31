#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy  # 导入ROS Python客户端库
import tf  # 导入tf库，用于处理坐标变换
from tf.transformations import euler_from_quaternion, translation_from_matrix  # 导入用于从四元数转换为欧拉角和从变换矩阵转换为平移向量的函数
import tf_conversions  # 导入tf_conversions库，用于转换tf数据
from geometry_msgs.msg import Twist  # 导入ROS的Twist消息类型
from nav_msgs.msg import Odometry  # 导入ROS的Odometry消息类型
import math  # 导入数学库

class robot_leader:
    def __init__(self):
        # 初始化ROS节点，设置节点名为'lidar_location'
        rospy.init_node('lidar_location', anonymous=False)
        # 从ROS参数服务器获取base_id、odom_id、map_id、leader_robot_name和robot_name参数
        self.baseId = rospy.get_param('~base_id', 'base_footprint')
        self.odomId = rospy.get_param('~odom_id', 'odom')
        self.mapId = rospy.get_param('~map_id', 'map')
        self.leader_robot_name = rospy.get_param('~leader_robot_name', 'robot_0')
        self.robot_name = rospy.get_param('~robot_name', 'robot_0')
        # 创建一个cmd_vel发布者
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # 创建一个odom订阅者，订阅名为'/leader_robot_name/odom'的主题
        rospy.Subscriber('/'+self.leader_robot_name+'/odom', Odometry, self.odomCB, queue_size=1)
        # 创建一个tf监听器
        self.tf_listener = tf.TransformListener()
        # 创建一个循环频率对象，设置循环频率为20Hz
        self.loop_rate = rospy.Rate(20)
        # 创建一个Twist对象，用于存储控制指令
        self.twist = Twist()
        # 初始化机器人位置和姿态
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.odom_yaw = 0.0
        # 初始化停止标志和领导者odom速度
        self.stop_mark = False
        self.leader_odom_vel_x = 0.0
        self.leader_odom_vel_y = 0.0
        # 初始化积分项
        self.prev_delta_x = 0.0
        self.integral_x = 0.0
        self.prev_delta_y = 0.0
        self.integral_y = 0.0
        self.prev_delta_yaw = 0.0
        self.integral_yaw = 0.0

    def odomCB(self, data):
        # 回调函数，当接收到odom数据时调用
        self.leader_odom_vel_x = data.twist.twist.linear.x
        self.leader_odom_vel_y = data.twist.twist.linear.y
        
 if __name__ == '__main__':
    robot_leader()
    rospy.spin()   