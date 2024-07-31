#!/usr/bin/env python
import rospy  # 导入ROS Python客户端库
import tf  # 导入tf库，用于处理坐标变换
from std_msgs.msg import UInt8  # 导入ROS的UInt8消息类型

car_type = 1  # 定义一个全局变量，用于存储车辆类型

def cartype_CB(data):
    global car_type
    if data.data != car_type:
        rospy.loginfo('Swich Car Type from %d to %d', car_type, data.data)
    car_type = data.data

def tf_transform():
    global car_type
    rospy.init_node('lidar_tf_transform', anonymous=False)  # 初始化ROS节点
    robot_name = rospy.get_param('~robot_name', '')  # 从ROS参数服务器获取robot_name参数
    x_pos = float(rospy.get_param('~x_pos', '0.0'))  # 从ROS参数服务器获取x_pos参数
    y_pos = float(rospy.get_param('~y_pos', '0.0'))  # 从ROS参数服务器获取y_pos参数
    angle = float(rospy.get_param('~angle', '0.0'))  # 从ROS参数服务器获取angle参数
    tf_broadcaster = tf.TransformBroadcaster()  # 创建一个tf广播器
    rate = rospy.Rate(50)  # 设置循环频率为50Hz
    rospy.loginfo('Start TF Transform')  # 打印日志信息
    while not rospy.is_shutdown():  # 循环，直到ROS节点关闭
        current_time = rospy.Time.now()  # 获取当前时间
        quat = tf.transformations.quaternion_from_euler(0, 0, angle)  # 创建一个四元数，表示旋转
        tf_broadcaster.sendTransform((x_pos, y_pos, 0.0), quat, current_time, robot_name + '/odom', '/map')  # 发送坐标变换
        rate.sleep()  # 等待，直到循环频率达到50Hz

if __name__ == '__main__':
    tf_transform()  # 调用tf_transform函数
