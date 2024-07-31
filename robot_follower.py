#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion, translation_from_matrix
import tf_conversions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class robot_leader:
  def __init__(self):
    rospy.init_node('lidar_location', anonymous=False)
    self.baseId = rospy.get_param('~base_id','base_footprint')
    self.odomId = rospy.get_param('~odom_id','odom')
    self.mapId = rospy.get_param('~map_id','map')
    self.leader_robot_name = rospy.get_param('~leader_robot_name','robot_0')
    self.robot_name = rospy.get_param('~robot_name','robot_0')
    self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/'+self.leader_robot_name+'/odom', Odometry, self.odomCB, queue_size=1)
    self.tf_listener = tf.TransformListener()
    self.loop_rate = rospy.Rate(20)
    self.twist = Twist()
    self.pos_x = 0.0
    self.pos_y = 0.0
    self.odom_yaw = 0.0
    self.stop_mark = False
    self.leader_odom_vel_x = 0.0
    while not rospy.is_shutdown():
      try: #get map position
        (trans,rot) = self.tf_listener.lookupTransform(self.robot_name+'/'+self.baseId, self.robot_name+'/follow_point', rospy.Time(0))
        euler = tf_conversions.transformations.euler_from_quaternion(rot)  #in radian system unit
        odom_yaw = ((euler[2]+6.28318)%6.28318)-3.14159 #comvert to map coordinate system
        odom_yaw = euler[2]  
        if math.sqrt(math.pow(trans[0],2) + math.pow(trans[1],2)) > 0.05:
          print("Delta1 %.02f, %.02f, %.02f %.02f"%(trans[0],trans[1],odom_yaw,math.atan2(trans[1],trans[0])))
          self.stop_mark = False
          delta_amgle = math.atan2(trans[1],trans[0])
          if abs(delta_amgle) > 0.05:
            self.twist.angular.z = 0.5*math.atan2(trans[1],trans[0])/abs(math.atan2(trans[1],trans[0]))
            if abs(delta_amgle) < 0.78: #abs 45 degree
              if abs(self.leader_odom_vel_x) < 0.01:
                self.twist.linear.x = 0.15
              else:
                self.twist.linear.x = self.leader_odom_vel_x
            else:
              self.twist.linear.x = 0
          else:
            if abs(self.leader_odom_vel_x) < 0.01:
              self.twist.linear.x = 0.15
            else:
              self.twist.linear.x = self.leader_odom_vel_x
            self.twist.angular.z = 0.0
          self.cmd_pub.publish(self.twist)    
        else:
          if abs(odom_yaw) > 0.05:
            self.stop_mark = False
            self.twist.angular.z = 0.5*odom_yaw/abs(odom_yaw)
            self.twist.linear.x = 0
            self.cmd_pub.publish(self.twist)    
          else:
            if not self.stop_mark:
              self.twist.angular.z = 0.0
              self.twist.linear.x = 0.0
              self.cmd_pub.publish(self.twist)
              self.stop_mark = True   
              rospy.loginfo("Follow Arrived")            
      except Exception as e:
        print(e)
        odom_yaw = 0.0                        
      self.loop_rate.sleep()
    self.twist.angular.z = 0.0
    self.twist.linear.x = 0.0
    self.cmd_pub.publish(self.twist)  
  def odomCB(self,data):
    self.leader_odom_vel_x = data.twist.twist.linear.x  

if __name__ == '__main__':
    robot_leader()
    rospy.spin()
#在这个循环中，robot_leader类订阅领导机器人的odom数据，并使用tf监听器来获取领导机器人和跟随点之间的变换。
# 它使用欧拉角转换和旋转矩阵转换来计算odom_yaw（yaw方向的角度）。
# 如果跟随点距离大于0.05米，它会计算当前角度与目标角度之间的差值，并根据这个差值来控制机器人的线速度和角速度。
# 如果跟随点距离小于0.05米, 它会根据odom_yaw来控制机器人的角速度。
# 如果跟随点到达目标位置，它会停止移动并打印一条日志信息。
#最后，odomCB函数用于处理odom数据，并更新领导机器人的odom速度。
#if __name__ == '__main__':部分是主函数，它创建robot_leader类的实例，并保持节点运行，直到接收到终止信号。
#这个节点的主要功能是订阅领导机器人的odom数据，并根据这些数据计算出跟随点的速度，然后发布自己的cmd_vel消息来控制自己的运动