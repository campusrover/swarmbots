#!/usr/bin/env python
import random, rospy, roslib

rospy.init_node("generate_random_x")
pos_range = float(rospy.get_param('pos_range', 3))

x_pos = random.uniform(-pos_range / 2, pos_range / 2)
rospy.set_param('map_merge/init_pose_x',x_pos)

print(x_pos)
