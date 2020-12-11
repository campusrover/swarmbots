#!/usr/bin/env python
import random, rospy, roslib

rospy.init_node("generate_random_position")

pos_range = float(rospy.get_param('pos_range', 3))
print(random.uniform(-pos_range / 2, pos_range / 2))
