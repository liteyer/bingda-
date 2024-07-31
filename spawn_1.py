#! /usr/bin/env python

import rospy
from rospy import client
from turtlesim.srv import Spawn,SpawnRequest,SpawnResponse

# 主函数入口
if __name__ == "__main__":
    # 初始化ROS节点，命名为"newturtle_p"
    rospy.init_node("newturtle_p")
    
    # 创建一个服务客户端，用于调用名为"/spawn"的服务
    client = rospy.ServiceProxy("/spawn",Spawn)
    
    # 创建一个SpawnRequest实例，用于传递请求参数
    request = SpawnRequest()
    
    # 设置请求参数：新海龟的初始位置和名称
    request.x = 4.5  # 新海龟在x轴的位置
    request.y = 2.0  # 新海龟在y轴的位置
    request.theta = 0  # 新海龟的初始角度
    request.name = "turtle2"  # 新海龟的名称
    
    # 等待服务端"/spawn"启动
    client.wait_for_service()
    
    # 尝试调用服务并处理可能的异常
    try:
        # 调用服务，并传入请求参数
        response = client.call(request)
        # 如果服务调用成功，打印成功信息
        rospy.loginfo("生成成功")
    except Exception as e:
        # 如果服务调用过程中发生异常，打印错误信息
        rospy.loginfo("wrong")
