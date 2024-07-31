#! /usr/bin/env python

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# 定义一个回调函数，用于接收turtle1的位姿信息
def turtle1callback(pose):
    # 将turtle1的位姿信息更新到理想位姿变量中
    ideal_pose.x = pose.x
    ideal_pose.y = pose.y
    ideal_pose.theta = pose.theta
    ideal_pose.linear_velocity = pose.linear_velocity
    ideal_pose.angular_velocity = pose.angular_velocity

# 定义一个回调函数，用于接收turtle2的位姿信息
def turtle2callback(pose):
    # 创建一个发布者，用于发布turtle2的速度控制命令
    pub = rospy.Publisher("/turtle2/cmd_vel",Twist,queue_size=1)
    
    # 计算turtle2相对于turtle1的x和y方向上的位置误差
    # 这里的计算方式可能是错误的，因为它试图将全局坐标误差转换为turtle2的本地坐标误差
    error_pose.x = (ideal_pose.x-pose.x)*math.cos(pose.theta) + (ideal_pose.y-pose.y)*math.sin(pose.theta) - 1
    error_pose.y = (ideal_pose.y-pose.y)*math.cos(pose.theta) - (ideal_pose.x-pose.x)*math.sin(pose.theta) - 1
    
    # 计算turtle2与turtle1之间的方向误差，并将其转换为弧度
    error_pose.theta = math.degrees(ideal_pose.theta) - math.degrees(pose.theta)
    error_pose.theta = math.radians(error_pose.theta)
    
    # 将理想位姿的速度信息赋值给误差位姿变量
    # 这些速度信息在后续的计算中并未使用
    error_pose.linear_velocity = ideal_pose.linear_velocity
    error_pose.angular_velocity = ideal_pose.angular_velocity
    
    # 创建一个Twist消息类型，用于存储速度控制命令
    twist = Twist()
    
    # 计算turtle2的线速度控制命令
    # 这里的计算公式可能需要调整以实现更好的跟随效果
    twist.linear.x = 2*math.cos(error_pose.theta) + 4 * error_pose.x
    
    # 计算turtle2的角速度控制命令
    # 同样，这里的计算公式可能需要调整
    twist.angular.z = 1 * ideal_pose.linear_velocity * error_pose.y + 1* error_pose.linear_velocity * math.sin(error_pose.theta)
    
    # 将线速度限制在最大值4以内
    if(abs(twist.linear.x)>4):
        twist.linear.x = 4
    
    # 发布速度控制命令给turtle2
    pub.publish(twist)

# 主函数，用于初始化ROS节点并设置订阅者
if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("ss1")
    
    # 初始化理想位姿和误差位姿变量
    ideal_pose = Pose()
    error_pose = Pose()
    
    # 订阅turtle1的位姿信息，并设置回调函数为turtle1callback
    sub1 = rospy.Subscriber("/turtle1/pose",Pose,turtle1callback,queue_size=10)
    
    # 订阅turtle2的位姿信息，并设置回调函数为turtle2callback
    sub2 = rospy.Subscriber("/turtle2/pose",Pose,turtle2callback,queue_size=10)
    
    # 保持节点运行，直到ROS节点被关闭
    rospy.spin()
