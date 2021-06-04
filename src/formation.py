#!/usr/bin/env python
import rospy
import array 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from mrs_msgs.srv import Vec4
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Float64Stamped
from mrs_msgs.msg import ReferenceStamped
from std_msgs.msg import Float64
import math
import numpy as np
import time
import os

class uav1class(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/uav1/odometry/odom_gps', Odometry, self.sub_callback)
        self.sub2 = rospy.Subscriber('/uav2/odometry/odom_gps', Odometry, self.sub_callback2)
        self.sub3 = rospy.Subscriber('/uav3/odometry/odom_gps', Odometry, self.sub_callback3)
        self.odomdata = Odometry()
        self.odomdata2 = Odometry()
        self.odomdata3 = Odometry()
        self.goal1 = [0.0,0.0,0.0,0.0]
        self.goal2 = [0.0,0.0,0.0,0.0]
        self.goal3 = [0.0,0.0,0.0,0.0]
        

    def sub_callback(self,msg):
        self.odomdata = msg
        #print (self.odomdata.pose.pose.position.z)

    def sub_callback2(self,msg):
        self.odomdata2 = msg

    def sub_callback3(self,msg):
        self.odomdata3 = msg

    def servicestarter(self):

        rospy.wait_for_service('/uav1/control_manager/goto_relative')
        rospy.wait_for_service('/uav2/control_manager/goto_relative')
        rospy.wait_for_service('/uav3/control_manager/goto_relative')
        try:
            self.calculator()
            service1 = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
            resp1 = service1(self.goal1)

            service2 = rospy.ServiceProxy('/uav2/control_manager/goto', Vec4)
            resp2 = service2(self.goal2)

            service3 = rospy.ServiceProxy('/uav3/control_manager/goto', Vec4)
            resp3 = service3(self.goal3)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def calculator(self):
        
        centroidx = (self.odomdata.pose.pose.position.x + self.odomdata2.pose.pose.position.x + self.odomdata3.pose.pose.position.x)/3
        centroidy = (self.odomdata.pose.pose.position.y + self.odomdata2.pose.pose.position.y + self.odomdata3.pose.pose.position.y)/3
        centroidz = (self.odomdata.pose.pose.position.z + self.odomdata2.pose.pose.position.z + self.odomdata3.pose.pose.position.z)/3

        self.goal2 = [centroidx,centroidy,centroidz,0.0]
        self.goal1 = [centroidx+5.0,centroidy,centroidz,0.0]
        self.goal3 = [centroidx-5.0,centroidy,centroidz,0.0]





def main():
    

    rospy.init_node('Formation_Controller', anonymous=True)
    rate = rospy.Rate(50)
    uav1object = uav1class()

    while not rospy.is_shutdown():
        uav1object.servicestarter()
        
        print (uav1object.odomdata)
        time.sleep(1)
        rate.sleep()

    
    #goal_sender()
    rospy.spin()

if __name__ == '__main__':
    main()

