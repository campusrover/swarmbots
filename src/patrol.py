#!/usr/bin/env python
import rospy
import actionlib
import tf2_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Pose
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from swarmbots.msg import State

# [CALLBACKS]

def state_callback(msg):
    global g_leader, g_leader_pose
    if msg.state == 'lead':
        g_leader = msg.robot_name
        g_leader_pose = msg.pose

def goal_pose():
  global g_leader, g_leader_pose
  goal_pose = MoveBaseGoal()
  
  # TODO: also need to check that map exists
  goal_pose = MoveBaseGoal()
  goal_pose.target_pose.header.frame_id = 'map'
  goal_pose.target_pose.pose = g_leader_pose
  return goal_pose


# A node called 'patrol' which is an action client to move_base
rospy.init_node('patrol')
robot_name = rospy.get_namespace()[1:-1]
g_leader = robot_name
g_leader_pose = Pose()
g_leader_pose.orientation.w = 1.0 # default

# [SUBSCRIBERS]

state_sub = rospy.Subscriber('/state', State, state_callback)


# Main program starts here
if __name__ == '__main__':
    
    rate = rospy.Rate(0.2)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # call on robot TODO: Change later to namespace
    client = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)

    print("starting patrol")
    # wait for action server to be ready
    client.wait_for_server()

    # Loop until ^c
    while not rospy.is_shutdown():
        print"leader", g_leader
        goal = goal_pose()
        print("Going for goal: ", goal)
        client.send_goal(goal)
        rate.sleep()