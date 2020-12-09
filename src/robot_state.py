#!/usr/bin/env python
import rospy, math, random
from swarmbots.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

# [CALLBACKS]
def state_callback(msg):
    global g_leader, g_leader_updated, g_follow
    # clear global lead/follow variables
    if msg.robot_name == g_leader:
        g_leader = None
    elif msg.robot_name == g_follow:
        g_follow = None
    # assign global vars
    if msg.state == 'lead':
        g_leader = msg.robot_name
        g_leader_updated = msg.header.stamp
    elif msg.state == 'follow':
        g_follow = msg.robot_name
    elif msg.state != 'dead':
        rospy.logerr('%s has unknown state %s', msg.robot_name, msg.state)

def odom_callback(msg):
    global g_positions, g_positions_updated
    if (rospy.Time.now() - g_positions_updated).to_sec() >= 1:
        g_positions = g_positions[1:] + [msg.pose.pose.position]
        g_positions_updated = rospy.Time.now()

# [HELPERS]
def is_moving():
    total_dist = 0
    prev_pos = None
    for pos in g_positions:
        if prev_pos is not None:
            total_dist += calc_xy_distance(prev_pos, pos)
        prev_pos = pos
    return total_dist > MIN_DIST_TRAVELED
    
def calc_xy_distance(point1, point2):
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

# [STATE VARIABLES]
robot_name = rospy.get_namespace()[1:-1]
g_leader = None
g_leader_updated = rospy.Time.from_sec(0)
g_follow = None
g_positions = [Point(), Point(), Point(), Point(), Point()]
g_positions_updated = rospy.Time.from_sec(0)

# [CONSTANTS]
MIN_DIST_TRAVELED = .3

# [MAIN CONTROL LOOP]
if __name__ == '__main__':
    rospy.init_node('robot_state')

    while rospy.get_rostime() == 0:
        pass
    
    # [PUBLISHERS]
    state_pub = rospy.Publisher('/state', State, queue_size=1)

    # [SUBSCRIBERS]
    state_sub = rospy.Subscriber('/state', State, state_callback)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        state_msg = State()
        state_msg.robot_name = robot_name
        state_msg.header.stamp = rospy.Time.now()

        if not is_moving():
            state_msg.state = 'dead'
            if g_leader == robot_name and g_follow is not None:
                # assign new leader
                leader_msg = State()
                leader_msg.robot_name = g_follow
                leader_msg.state = 'lead'
                state_pub.publish(leader_msg)
        elif g_leader is None or g_leader == robot_name or ((rospy.Time.now() - g_leader_updated).to_sec() >= 5 and g_leader != robot_name):
            state_msg.state = 'lead'
        else:
            state_msg.state = 'follow'

        
        state_pub.publish(state_msg)
        rate.sleep()