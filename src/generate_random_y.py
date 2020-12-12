#!/usr/bin/env python
import random, rospy, roslib

rospy.init_node("generate_random_y")
pos_range = float(rospy.get_param('pos_range', 3))

y_pos = random.uniform(-pos_range / 2, pos_range / 2)
rospy.set_param('map_merge/init_pose_y', y_pos)

print(y_pos)