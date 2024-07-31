#! /usr/bin/env python

import rospy
from rospy import client
from turtlesim.srv import Spawn,SpawnRequest,SpawnResponse

# 主函数入口
if __name__ == "__main__":
    # 初始化ROS节点，节点名称为"newturtle_p"
    rospy.init_node("newturtle_p")
    
    # 创建一个服务客户端，用于连接名为"/spawn"的服务
    # 这个服务用于在turtlesim中创建一个新的海龟
    client = rospy.ServiceProxy("/spawn",Spawn)
    
    # 创建一个SpawnRequest实例，用于定义新海龟的生成参数
    request = SpawnRequest()
    
    # 设置新海龟的初始位置和名称
    request.x = 1.0  # 新海龟在x轴的位置
    request.y = 1.0  # 新海龟在y轴的位置
    request.theta = 0  # 新海龟的初始朝向角度，0表示朝向x轴正方向
    request.name = "turtle3"  # 新海龟的名称
    
    # 等待直到"/spawn"服务可用
    client.wait_for_service()
    
    # 尝试调用服务，可能会抛出异常
    try:
        # 调用服务，生成新海龟，并接收响应
        response = client.call(request)
        # 如果服务调用成功，输出日志信息
        rospy.loginfo("生成成功")
    except Exception as e:
        # 如果在调用服务过程中发生异常，输出错误日志信息
        rospy.loginfo("wrong")
