#!/usr/bin/env python
import rospy, math, random
from swarmbots.msg import Status
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

# [CALLBACKS]
def status_callback(msg):
    global g_leader, g_leader_updated, g_free
    if msg.status == 'leader':
        g_leader = msg.robot_name
        g_leader_updated = msg.header.stamp
    elif msg.status == 'free':
        g_free = msg.robot_name
    elif msg.status == 'dead':
        if msg.robot_name == g_leader:
            g_leader = None
        elif msg.robot_name == g_free:
            g_free = None
    else:
        rospy.logerr('%s has unknown status %s', msg.robot_name, msg.status)

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
g_free = None
g_positions = [Point(), Point(), Point(), Point(), Point()]
g_positions_updated = rospy.Time.from_sec(0)

# [CONSTANTS]
MIN_DIST_TRAVELED = .3

# [MAIN CONTROL LOOP]
if __name__ == '__main__':
    rospy.init_node('robot_status')

    while rospy.get_rostime() == 0:
        pass
    
    # [PUBLISHERS]
    status_pub = rospy.Publisher('/status', Status, queue_size=1)

    # [SUBSCRIBERS]
    status_sub = rospy.Subscriber('/status', Status, status_callback)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        status_msg = Status()
        status_msg.robot_name = robot_name
        status_msg.header.stamp = rospy.Time.now()

        if g_free is None:
            # if all robots are dead, make them leaders to give them a place to go (and hopefully get unstuck)
            status_msg.status = 'leader'
        elif not is_moving():
            status_msg.status = 'dead'
            if g_leader == robot_name:
                # assign new leader
                leader_msg = Status()
                leader_msg.robot_name = g_free
                leader_msg.status = 'leader'
                status_pub.publish(leader_msg)
        elif g_leader is None or g_leader == robot_name or ((rospy.Time.now() - g_leader_updated).to_sec() >= 5 and g_leader != robot_name):
            status_msg.status = 'leader'
        else:
            status_msg.status = 'free'

        
        status_pub.publish(status_msg)
        rate.sleep()