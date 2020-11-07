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
    t.child_frame_id = robot_name + '_tf/base_link'
    t.transform.translation = msg.pose.pose.position
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    robot_name = rospy.get_param('~robot_name')
    rospy.Subscriber('/%s/odom' % robot_name,
                     Odometry,
                     handle_robot_pose,
                     robot_name)
    rospy.spin()