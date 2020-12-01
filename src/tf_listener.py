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
    global g_angular_vel, g_linear_vel
    g_angular_vel = msg.twist.twist.angular.z
    g_linear_vel = msg.twist.twist.linear.x

# [HELPERS]
def calc_magnitude(vector):
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)

def print_state():
    print('*** ' + robot_name + ' ***')
    print('State: ' + g_state)
    print('Linear vel: ' + str(g_linear_vel))
    print('Angular vel: ' + str(g_angular_vel))
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

    # calculate distance
    dist =  math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

    # if calc_magnitude(trans.transform.translation) > .8:

    msg.linear.x = 0.5 * dist
    # make it not go too fast... gotta go slow
    # commenting out for now bc it doesn't go around corners good without some wall avoidance
    msg.linear.x = min(0.2, msg.linear.x)

    # calculate angle
    # this one turns them in the direction away from the leader.
    avoid_angle = 4 * math.atan2(-trans.transform.translation.y, -trans.transform.translation.x)
    # this is the following direction
    approach_angle = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
    

    # meters to stay apart
    SOCIAL_DISTANCE = 0.5
    # stay away if approaching subject closer than social distancing meters!
    if dist < SOCIAL_DISTANCE:
        msg.angular.z = avoid_angle
    # else if it's about the same dist, stay put
    elif round(dist, 1) == SOCIAL_DISTANCE:
        msg.angular.z = 0
        msg.linear.x = 0
    # continue to follow from a safe distance
    else:
        msg.angular.z = approach_angle

    
    return msg
    
def avoid_obstacle():
    global g_state
    # use map if exists
    # if not, wall-follow
    # if g_ranges['FL'] < MIN_WALL_DIST and g_ranges['FR'] < MIN_WALL_DIST:
    print('avoid obstacle?!')
    return Twist()
    
def lead():
    msg = Twist()
    # check whether anything is too close
    if g_ranges['F'] < MIN_WALL_DIST:
        msg.angular.z = MAX_ANGULAR_VEL
    else: # drive forward
        msg.linear.x = MAX_LINEAR_VEL
    return msg

# [STATE VARIABLES]
g_state = 'follow' # [follow, avoid obstacle, lead]
g_ranges = {
    'F': float('inf'), # [355:] + [:5]
    'L': float('inf'), # [16:105]
    'R': float('inf'), # [255:344]
    'FL': float('inf'), # [6:15]
    'FR': float('inf') # [345:354]
}
g_angular_vel = 0
g_linear_vel = 0

# [CONSTANTS]
MAX_LINEAR_VEL = .4
MAX_ANGULAR_VEL = math.pi/4
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

    # get initial state
    g_state = rospy.get_param('~state', 'follow')

    robot_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print_state()

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