#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

import math

actual_ = Point()
rot_ = 0
# machine state
state_ = 0
# goal
destino = Point()
destino.x = -3
destino.y = -5
destino.z = 0

pub = None

def llegada(goal):
    e_rot =math.atan2(goal.y-actual_.y,goal.x-actual_.x)-rot_
    e_des = goal.y-actual_.y
    print(actual_)

    twist_msg = Twist()
    if math.fabs(e_rot) > 0:
        twist_msg.angular.z = -2 if e_rot > 0 else 2
    
    if e_des > 0:
        twist_msg.linear.x = 2
    elif e_des<0:
        twist_msg.linear.x = -2
    else:    
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
       
    pub.publish(twist_msg)

def clbk_odom(msg):
    global actual_
    # position
    actual_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def main():
    global pub
    
    rospy.init_node('destino')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        llegada(destino)
        rate.sleep()

if __name__ == '__main__':
    main()