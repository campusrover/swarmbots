#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# [CALLBACKS]
def scan_callback(msg):
    # todo: make this work for any lidar. this depends on 1 value per degree
    g_ranges['F'] = min(msg.ranges[355:] + msg.ranges[:6])
    g_ranges['FL'] = min(msg.ranges[6:16])
    g_ranges['L'] = min(msg.ranges[16:105])
    g_ranges['R'] = min(msg.ranges[255:345])
    g_ranges['FR'] = min(msg.ranges[345:355])

def odom_callback(msg):
    print('odom callback')

# [HELPERS]
def calc_magnitude(vector):
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)

def print_state():
    print('*** ' + robot_name + ' ***')
    print('State: ' + g_state)
    print('Ranges:')
    print('\tF: ' + str(g_ranges['F']))
    print('\tL: ' + str(g_ranges['L']))
    print('\tFL: ' + str(g_ranges['FL']))
    print('\tFR: ' + str(g_ranges['FR']))
    print('\tR: ' + str(g_ranges['R']))

# [STATE FUNCTIONS]
def follow():
    try:
        trans = tfBuffer.lookup_transform(robot_name + '/odom', 'swarmboss/odom', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return # exceptions may happen once in a while, so don't override existing Twist command

    msg = Twist()

    if calc_magnitude(trans.transform.translation) > .8:
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        # make it not go too fast... gotta go slow
        # commenting out for now bc it doesn't go around corners good without some wall avoidance
        msg.linear.x = min(0.5, msg.linear.x)
    
    return msg
    
def avoid_obstacle():
    global g_state
    # use map if exists
    # if not, wall-follow
    # if g_ranges['FL'] < MIN_WALL_DIST and g_ranges['FR'] < MIN_WALL_DIST:
    # here
    return Twist()
    
def lead():
    return Twist()

# [STATE VARIABLES]
g_state = 'follow' # [follow, avoid obstacle, lead]
g_ranges = {
    'F': float('inf'), # [355:] + [:5]
    'L': float('inf'), # [16:105]
    'R': float('inf'), # [255:344]
    'FL': float('inf'), # [6:15]
    'FR': float('inf') # [345:354]
}

# [CONSTANTS]
MIN_WALL_DIST = .4

# [SUBSCRIBERS]
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

# [PUBLISHERS]
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# [MAIN CONTROL LOOP]
if __name__ == '__main__':
    rospy.init_node('tf_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    robot_name = rospy.get_namespace()[1:-1]
    # remove this depending on if you have tf prefix
    # robot_name += '_tf'
    robot_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print_state()

        if min(g_ranges['F']) < MIN_WALL_DIST:
            g_state = 'avoid obstacle'
        elif min(g_ranges.values()) >= MIN_WALL_DIST:
            g_state = 'follow'

        if g_state == 'follow':
            msg = follow()
        elif g_state == 'avoid obstacle':
            msg = avoid_obstacle()
        elif g_state == 'lead':
            msg = lead()
        else:
            print('state not recognized')
        
        if msg is not None: # do not override previous cmd_vel if msg is None
            cmd_vel_pub.publish(msg)

        rate.sleep()