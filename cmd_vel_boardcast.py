#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
from geometry_msgs.msg import Twist

# 定义一个回调函数，用于处理接收到的Twist消息
def cmd_callback(data):
    # 循环遍历每个机器人
    for i in range(robot_num):
        # 创建一个新的Twist对象
        twist = Twist()
        # 将接收到的Twist消息赋值给当前的twist对象
        twist = data
        # 尝试访问Twist对象的angular字段，但这个操作似乎是多余的，因为没有使用它
        twist.angular
        # 使用names字典中对应于当前机器人的发布者发布twist消息
        names['cmd_pub_%s' % i].publish(twist)

# 主函数，程序的入口点
if __name__ == '__main__':
    # 尝试初始化ROS节点
    try:
        # 初始化名为'cmd_vel_boardcast'的ROS节点
        rospy.init_node('cmd_vel_boardcast')
        # 从ROS参数服务器获取机器人数量，默认为1
        robot_num = int(rospy.get_param('~robot_num', '1'))
        if robot_num < 1:
            robot_num = 1
        # 从ROS参数服务器获取twist命令主题，默认为'/cmd_vel'
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        # 创建一个订阅者，订阅名为'/cmd_vel'的主题，当接收到消息时调用cmd_callback函数
        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
        # 创建一个信息字符串，包含节点运行信息和机器人数量
        info_string = "cmd_vel_boardcast running, robot_num: %d" % robot_num
        # 使用ROS日志系统打印出信息字符串
        rospy.loginfo(info_string)
        # 创建一个字典，用于存储创建的发布者和订阅者对象
        names = locals()
        # 循环遍历每个机器人，创建发布者并将其添加到names字典中
        for i in range(robot_num):
            twist_cmd_topic_name = 'robot_%s%s' % (i, twist_cmd_topic)
            names['cmd_pub_%s' % i] = rospy.Publisher(twist_cmd_topic_name, Twist, queue_size=1)
        # 保持节点运行，直到接收到终止信号
        rospy.spin()

    # 捕获ROSInterruptException异常，这是ROS节点关闭时会抛出的异常
    except rospy.ROSInterruptException:
        # 如果捕获到异常，则什么都不做，允许节点正常关闭
        pass
