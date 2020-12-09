#!/usr/bin/env python
import rospy
import actionlib
import tf2_ros

# move_base is the package that takes goals for navigation
# there are different implemenetations with a common interface
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from swarmbots.msg import Status

# [CALLBACKS]
def odom_callback(msg):
    global g_pose_stamped
    g_pose_stamped = msg.pose

def status_callback(msg):
    global g_leader
    if msg.status == 'leader':
        g_leader = msg.robot_name

# You need to know the coordinates that the map you are working
# in is. I experimented with these numbers and the turtlebot3_stage_4
# map from Turtlebot3. The first array is x,y,z location. The second one
# is a "quaternion" defining an orientation. Quaternions are a different
# mathematical represetnation for "euler angles", yaw, pitch and roll.

waypoints = [
    [ (-1.0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)],
    [ (-1.0, 2.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
]

# Function to generate a proper MoveBaseGoal() from a two dimensional array
# containing a location and a rotation. This is just to make the waypoints array
# simpler.

def goal_pose(pose):
  global g_leader, g_pose
  goal_pose = MoveBaseGoal()
  try:
      trans = tfBuffer.lookup_transform('robot1/odom', 'robot0/odom', rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return # exceptions may happen once in a while, so don't override existing Twist command
  
  # set goal pose as leader position TODO: make it stay 1m away from leader
  new_pose_stamped = tf2_geometry_msgs.do_transform_pose(g_pose_stamped, trans)
  # TODO: also need to check that map exists
  goal_pose = MoveBaseGoal()
  goal_pose.target_pose.header.frame_id = 'map_merge/map'
  goal_pose.target_pose.pose = new_pose_stamped.pose
  
  return goal_pose



# [SUBSCRIBERS]
odom_sub = rospy.Subscriber('robot1/odom', Odometry, odom_callback)
status_sub = rospy.Subscriber('robot1/status', Status, status_callback)

robot_name = rospy.get_namespace()[1:-1]
g_leader = robot_name

# Main program starts here
if __name__ == '__main__':

    # A node called 'patrol' which is an action client to move_base
    rospy.init_node('patrol')

    rate = rospy.Rate(0.1)

    g_pose_stamped = None
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    client = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)

    print("starting patrol")

    # wait for action server to be ready
    client.wait_for_server()

    # Loop until ^c
    while not rospy.is_shutdown():

        # repeat the waypoints over and over again
        for pose in waypoints:
            goal = goal_pose(pose)
            print("Going for goal: ", goal)
            client.send_goal(goal)
            rate.sleep()