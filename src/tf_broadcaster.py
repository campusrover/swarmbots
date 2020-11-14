#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry


def handle_robot_pose(msg, robot_name):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = robot_name + '/map'
    t.transform.translation = msg.pose.pose.position
    t.transform.rotation = msg.pose.pose.orientation

    try:
        # tf prefix
        trans = tfBuffer.lookup_transform(robot_name + '/odom', robot_name + '/map', rospy.Time())
        print robot_name, "found map to odom tf"
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        t.child_frame_id = robot_name + '/odom'
        print e
    
    br.sendTransform(t)



rospy.init_node('tf_broadcaster')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
robot_name = rospy.get_namespace()[1:-1]
# remove this depending on if you have tf prefix
# robot_name += '_tf'
rospy.Subscriber('odom',
                     Odometry,
                     handle_robot_pose,
                     robot_name)

if __name__ == '__main__':
    rospy.spin()