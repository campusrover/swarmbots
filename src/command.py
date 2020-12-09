#!/usr/bin/env python  
import rospy
from std_msgs.msg import String

rospy.init_node('user_commands')

pub = rospy.Publisher('/command', String, queue_size='1', latch=True)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        command_msg = String
        command = str(raw_input("Enter next command [follow, disperse]: ")).strip(' ')
        
        if command == 'follow':
            command_msg = command
        elif command == 'disperse':
            command_msg = command
        else:
            print("Unrecognized command.")
            continue
        pub.publish(command_msg)
        print 'published command', command_msg