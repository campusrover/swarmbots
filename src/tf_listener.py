#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import actionlib
import geometry_msgs.msg
from geometry_msgs.msg import Twist, PoseStamped, Pose
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from swarmbots.msg import State
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# [CALLBACKS]
def scan_callback(msg):
    global wall_ahead
    # todo: make this work for any lidar. this depends on 1 value per degree
    g_ranges['F'] = min(msg.ranges[355:] + msg.ranges[:6])
    g_ranges['FL'] = min(msg.ranges[6:16])
    g_ranges['L'] = min(msg.ranges[16:105])
    g_ranges['R'] = min(msg.ranges[255:345])
    g_ranges['FR'] = min(msg.ranges[345:355])
    if g_ranges['F'] < MIN_WALL_DIST or g_ranges['FL'] < MIN_WALL_DIST or g_ranges['FR'] < MIN_WALL_DIST:
        wall_ahead = True
    else:
        wall_ahead = False

def odom_callback(msg):
    global g_angular_vel, g_linear_vel
    g_angular_vel = msg.twist.twist.angular.z
    g_linear_vel = msg.twist.twist.linear.x

def state_callback(msg):
    global g_leader, g_state
    if msg.state == 'lead':
        g_leader = msg.robot_name
    if msg.robot_name == robot_name:
        g_state = msg.state

def command_callback(msg):
    global current_command
    current_command = msg.data


# [HELPERS]

def print_state(): # monitor values for ranges that the robots are receiving
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
    print('\tB: ' + str(g_ranges['B']))
    
def calc_dist_avoid_approach(trans): # helper fn to calculate variables for follow() and disperse() from transformation
    global MAX_ANGULAR_VEL
    # calculate distance between leader and robot
    dist =  math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
    
    # this rotation turns them in the direction away from the leader.
    avoid_angle = 4 * math.atan2(-trans.transform.translation.y, -trans.transform.translation.x)
    if avoid_angle >= 0:
        avoid_angle = min(abs(MAX_ANGULAR_VEL), abs(avoid_angle))
    else:
        avoid_angle = -1 * min(abs(MAX_ANGULAR_VEL), abs(avoid_angle))
    # this is the following direction
    approach_angle = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
    if approach_angle >= 0:
        approach_angle = min(abs(MAX_ANGULAR_VEL), abs(approach_angle))
    else:
        approach_angle = -1 * min(abs(MAX_ANGULAR_VEL), abs(approach_angle))
    return dist, avoid_angle, approach_angle

# [STATE FUNCTIONS]
def follow():
    global MAX_LINEAR_VEL, SOCIAL_DIST
    # if can't move past wall ahead, default to wander state
    if wall_ahead:
        return wander()
    try:
        trans = tfBuffer.lookup_transform(robot_name + '/odom', g_leader + '/odom', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return # exceptions may happen once in a while, so don't override existing Twist command
    
    dist, avoid_angle, approach_angle = calc_dist_avoid_approach(trans)
    print dist, avoid_angle, approach_angle
    
    msg = Twist()
    msg.linear.x = min(MAX_LINEAR_VEL*.75, 0.5 * dist)
    # stay away if approaching subject closer than social distancing meters!
    if dist < SOCIAL_DIST:
        msg.angular.z = avoid_angle
    # else if it's about the same dist, stay put
    elif round(dist, 1) == SOCIAL_DIST:
        msg.angular.z = 0
        msg.linear.x = 0
    # continue to follow from a safe distance
    else:
        msg.angular.z = approach_angle
    return msg
    

def wander():
    vel_msg = Twist()
    # check if anything in front or back is too close to turn properly
    if min(g_ranges['F'], g_ranges['FL'], g_ranges['FR']) < TOO_CLOSE_TO_TURN:
        vel_msg.linear.x = -MAX_LINEAR_VEL
    elif g_ranges['B'] < TOO_CLOSE_TO_TURN:
        vel_msg.linear.x = MAX_LINEAR_VEL
    elif wall_ahead: # check if anything in front is too close (will collide if continue moving)
        vel_msg.angular.z = MAX_ANGULAR_VEL
    else: # drive forward
        vel_msg.linear.x = MAX_LINEAR_VEL
    return vel_msg


# robot tries to stay far away from leader, while exploring the world
# if significantly far away
def disperse():
    global DISPERSE_DIST, MAX_LINEAR_VEL
    # if can't move past wall ahead, default to wander state
    if wall_ahead:
        return wander()
    try:
        trans = tfBuffer.lookup_transform(robot_name + '/odom', g_leader + '/odom', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return # exceptions may happen once in a while, so don't override existing Twist command
    
    dist, avoid_angle, approach_angle = calc_dist_avoid_approach(trans)
    msg = Twist()

    # stay away if approaching subject closer than social distancing meters!
    if dist < DISPERSE_DIST:
        msg.angular.z = avoid_angle
        msg.linear.x = MAX_LINEAR_VEL
    # else if it's about the same dist, stay put
    elif round(dist, 1) == DISPERSE_DIST:
        msg.angular.z = 0
        msg.linear.x = 0
    # disperse and wander
    else:
        return wander()
    return msg

# [INITIALIZE NODE]
rospy.init_node('tf_listener')

# [STATE VARIABLES]
robot_name = rospy.get_namespace()[1:-1]
g_leader = robot_name # keep track of which robot is leader
g_state = 'follow' # [follow, wander, lead]
g_ranges = {
    'F': float('inf'), # [355:] + [:5]
    'L': float('inf'), # [16:105]
    'R': float('inf'), # [255:344]
    'FL': float('inf'), # [6:15]
    'FR': float('inf'), # [345:354]
    'B': float('inf') # [105:255]
}
g_angular_vel = 0 # current robot's angular and linear velocities
g_linear_vel = 0
wall_ahead = False # keep track of if wall is ahead of robot
current_command = 'follow' # default command set to follow

# [CONSTANTS]
MAX_LINEAR_VEL = .3
MAX_ANGULAR_VEL = math.pi/4
MIN_WALL_DIST = .4
TOO_CLOSE_TO_TURN = .15
SOCIAL_DIST = 0.5 # meters to stay apart when following the leader
DISPERSE_DIST = 4 # meters to stay apart when dispersing

# [SUBSCRIBERS]
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
state_sub = rospy.Subscriber('/state', State, state_callback)
command_sub = rospy.Subscriber('/command', String, command_callback)

# [PUBLISHERS]
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
client = actionlib.SimpleActionClient(robot_name + '/move_base', MoveBaseAction) # move base action client

# [MAIN CONTROL LOOP]
if __name__ == '__main__':

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        # print_state()

        if g_state == 'follow':
            if current_command == 'disperse':
                msg = disperse()
            else:
                msg = follow()
        elif g_state == 'lead':
            msg = wander()
        elif g_state == 'dead':
            msg = wander()
        else:
            rospy.logerr('%s has unknown state %s', robot_name, g_state)
        
        if msg is not None: # do not override previous cmd_vel if msg is None
            cmd_vel_pub.publish(msg)
        rate.sleep()
