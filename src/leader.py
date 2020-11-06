#!/usr/bin/env python

import rospy, random, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# [CALLBACKS]
def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges[330:] + msg.ranges[:30])

def odom_callback(msg):
    global g_angular_vel, g_linear_vel
    g_angular_vel = msg.twist.twist.angular.z
    g_linear_vel = msg.twist.twist.linear.x

# [HELPERS]
def print_state():
    print('*** ' + name + ' ***')
    print('Range ahead: ' + str(g_range_ahead))
    print('Linear vel: ' + str(g_linear_vel))
    print('Angular vel: ' + str(g_angular_vel))

# [STATE VARIABLES]
g_range_ahead = 1
g_angular_vel = 0
g_linear_vel = 0

# [CONSTANTS]
MAX_LINEAR_VEL = .4
MAX_ANGULAR_VEL = math.pi/4
MIN_WALL_DIST = .4
name = 'swarmboss'

rospy.init_node(name)
rate = rospy.Rate(10)

# [SUBSCRIBERS]
scan_sub = rospy.Subscriber(name + '/scan', LaserScan, scan_callback)
odom_sub = rospy.Subscriber(name + '/odom', Odometry, odom_callback)

# [PUBLISHERS]
cmd_vel_pub = rospy.Publisher(name + '/cmd_vel', Twist, queue_size=1)

# [MAIN CONTROL LOOP]
while not rospy.is_shutdown():
    print_state()

    # check whether anything is too close
    if g_range_ahead < MIN_WALL_DIST:
        g_linear_vel = 0
        g_angular_vel = MAX_ANGULAR_VEL
    else: # drive forward
        g_linear_vel = MAX_LINEAR_VEL
        g_angular_vel = 0
    
    # create and publish twist message
    twist = Twist()
    twist.linear.x = g_linear_vel
    twist.angular.z = g_angular_vel
    cmd_vel_pub.publish(twist)

    rate.sleep()