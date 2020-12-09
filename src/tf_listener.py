#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import actionlib
import geometry_msgs.msg
from geometry_msgs.msg import Twist, PoseStamped
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
<<<<<<< Updated upstream
from swarmbots.msg import State
=======
from swarmbots.msg import Status
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
>>>>>>> Stashed changes

# [CALLBACKS]
def scan_callback(msg):
    # todo: make this work for any lidar. this depends on 1 value per degree
    g_ranges['F'] = min(msg.ranges[355:] + msg.ranges[:6])
    g_ranges['FL'] = min(msg.ranges[6:16])
    g_ranges['L'] = min(msg.ranges[16:105])
    g_ranges['R'] = min(msg.ranges[255:345])
    g_ranges['FR'] = min(msg.ranges[345:355])

def odom_callback(msg):
    global g_angular_vel, g_linear_vel, g_pose_stamped
    g_angular_vel = msg.twist.twist.angular.z
    g_linear_vel = msg.twist.twist.linear.x
    g_pose_stamped = msg.pose

def state_callback(msg):
    global g_leader, g_state
    if msg.state == 'lead':
        g_leader = msg.robot_name
    if msg.robot_name == robot_name:
        g_state = msg.state

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
    global tfBuffer
    try:
        trans = tfBuffer.lookup_transform(robot_name + '/odom', g_leader + '/odom', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return # exceptions may happen once in a while, so don't override existing Twist command

    msg = Twist()
    # calculate distance
    dist =  math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
    msg.linear.x = 0.5 * dist
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
    global g_state, robot_name, client, tfBuffer
    # use map if exists
    try:
        trans = tfBuffer.lookup_transform(robot_name + '/odom', g_leader + '/odom', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return # exceptions may happen once in a while, so don't override existing Twist command
    
    # set goal pose as leader position TODO: make it stay 1m away from leader
    new_pose_stamped = tf2_geometry_msgs.do_transform_pose(g_pose_stamped, trans)
    # TODO: also need to check that map exists
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map_merge/map'
    goal_pose.target_pose.pose = new_pose_stamped.pose
    
    # idk how this works
    client.send_goal(goal_pose)
    # client.wait_for_result()
    
    
    # if not, wall-follow
    # if g_ranges['FL'] < MIN_WALL_DIST and g_ranges['FR'] < MIN_WALL_DIST:
    print('avoid obstacle?!')
    return Twist()

def wander():
    vel_msg = Twist()
    # check whether anything is too close
    if g_ranges['F'] < MIN_WALL_DIST:
        vel_msg.angular.z = MAX_ANGULAR_VEL
    else: # drive forward
        vel_msg.linear.x = MAX_LINEAR_VEL
    return vel_msg

# [INITIALIZE NODE]
rospy.init_node('tf_listener')

# [STATE VARIABLES]
robot_name = rospy.get_namespace()[1:-1]
g_leader = robot_name
g_state = 'follow' # [follow, wander, lead]
g_ranges = {
    'F': float('inf'), # [355:] + [:5]
    'L': float('inf'), # [16:105]
    'R': float('inf'), # [255:344]
    'FL': float('inf'), # [6:15]
    'FR': float('inf') # [345:354]
}
g_angular_vel = 0
g_linear_vel = 0
g_pose_stamped = PoseStamped()

# [CONSTANTS]
MAX_LINEAR_VEL = .4
MAX_ANGULAR_VEL = math.pi/4
MIN_WALL_DIST = .6

# [SUBSCRIBERS]
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
state_sub = rospy.Subscriber('/state', State, state_callback)

# [PUBLISHERS]
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
client = actionlib.SimpleActionClient(robot_name + '/move_base', MoveBaseAction) # move base action client
# client.wait_for_server() # wait for it to start ig

# [MAIN CONTROL LOOP]
if __name__ == '__main__':

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # print_state()

        if g_state == 'follow':
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