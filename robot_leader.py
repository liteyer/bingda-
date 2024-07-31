#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion, translation_from_matrix
import tf_conversions
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import String

class robot_leader:
    def __init__(self):
        rospy.init_node('robot_leader', anonymous=False)
        self.baseId = rospy.get_param('~base_id','base_footprint')
        self.odomId = rospy.get_param('~odom_id','odom')
        self.mapId = rospy.get_param('~map_id','map')
        self.group_type = rospy.get_param('~group_type','triangle')
        self.robot_num = int(rospy.get_param('~robot_num','5'))    
        self.robot_gap = float(rospy.get_param('~robot_gap','0.5'))    
        # self.cmd_pub = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("group_type", String, self.group_typeCB, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.loop_rate = rospy.Rate(50)
        rospy.loginfo("robot_leader Node Start")
        while not rospy.is_shutdown():
          if self.group_type == "line_x":
            quat = tf.transformations.quaternion_from_euler(0,0,0)
            for i in range(1,self.robot_num):
              child_frame_name = 'robot_' + str(i) + '/follow_point'
              self. tf_broadcaster.sendTransform((-self.robot_gap*i,0.0,0.0),quat,rospy.Time.now(),child_frame_name,'/robot_0/base_footprint') 
          if self.group_type == "line_y":
            quat = tf.transformations.quaternion_from_euler(0,0,0)
            for i in range(1,self.robot_num):
              child_frame_name = 'robot_' + str(i) + '/follow_point'
              self. tf_broadcaster.sendTransform((0.0,-self.robot_gap*i,0.0),quat,rospy.Time.now(),child_frame_name,'/robot_0/base_footprint')               
          if self.group_type == "triangle":
            quat = tf.transformations.quaternion_from_euler(0,0,0)
            for i in range(self.robot_num):
              child_frame_name = 'robot_' + str(i) + '/follow_point'
              if i == 1:
                self. tf_broadcaster.sendTransform((-self.robot_gap,self.robot_gap/2.0,0.0),quat,rospy.Time.now(),child_frame_name,'/robot_0/base_footprint')  
              elif i == 2:
                self. tf_broadcaster.sendTransform((-self.robot_gap,-self.robot_gap/2.0,0.0),quat,rospy.Time.now(),child_frame_name,'/robot_0/base_footprint')       
              if i == 3:
                self. tf_broadcaster.sendTransform((-self.robot_gap*2.0,self.robot_gap,0.0),quat,rospy.Time.now(),child_frame_name,'/robot_0/base_footprint')  
              elif i == 4:
                self. tf_broadcaster.sendTransform((-self.robot_gap*2.0,-self.robot_gap,0.0),quat,rospy.Time.now(),child_frame_name,'/robot_0/base_footprint')  
          self.loop_rate.sleep()
    def group_typeCB(self,data):
       self.group_type = data.data
       rospy.loginfo("Switch to %s Group",self.group_type)

if __name__ == '__main__':
    robot_leader()
    rospy.spin()
