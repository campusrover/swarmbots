#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg

def calc_magnitude(vector):
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    robot_name = rospy.get_namespace()[1:-1]
    robot_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(robot_name + '_tf/base_link', 'swarmboss_tf/base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()

        if calc_magnitude(trans.transform.translation) > .8:
            msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        robot_vel.publish(msg)

        rate.sleep()