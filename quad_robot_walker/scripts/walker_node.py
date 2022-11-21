#!/usr/bin/env python
#credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function

import roslib; roslib.load_manifest('champ_teleop')
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point 
from tf.transformations import euler_from_quaternion
from math import atan2
import random

import sys, select, termios, tty
import numpy as np

import tf

x = 0.0
y = 0.0
theta = 0.0 

strmsg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
q: one time square of side 5 unit
w: continous square of side 5 unit
a: one time circle of radius 5 unit
s: continous circle of radius 5 unit
"""

def newOdom (msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("champ_walker_node")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(2)

goal = Point()
goal.x = 0
goal.y = 0

try :
    print(strmsg)

    while not rospy.is_shutdown():
        inc_x = goal.x - x
        inc_y = goal.y - y 

        angle_to_goal = atan2(inc_y, inc_x)
        angl = str(angle_to_goal - theta)
        print("target:\tx %s\ty %s, currently:\tx %s\ty %s \tangl %s " % (goal.x,goal.y, x,y,angl))
        
        if ((abs(angle_to_goal - theta) > 0.5)):    
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            pub.publish(speed)
            r.sleep()
        elif ((abs(inc_x) > 0.1) or (abs(inc_y) > 0.1)):
            speed.linear.x = 0.5
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            goal.x = random.randrange(0,5)
            goal.y = random.randrange(0,5)

#        pub.publish(speed)
#        r.sleep()
except Exception as e:
    print(e)

finally:
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0