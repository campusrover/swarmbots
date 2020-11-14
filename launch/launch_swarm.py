#!/usr/bin/env python
import rospy, roslaunch

# def analyze_args(cli_args):
#     parsed = dict()
#     for arg in cli_args:
#         if ':=' in arg:
#             idx = arg.index(':=')
#             parsed[arg[:idx]] = arg[idx+2:]
#     return parsed

# parsed_args = analyze_args(sys.argv)
# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)

# model_arg = 'model:=' + parsed_args.get('model', 'burger_for_autorace')
# followers = parsed_args.get('followers', 2)

# main_launch = ['swarmbots', 'main.launch', model_arg]
# launch_files = [(roslaunch.rlutil.resolve_launch_arguments(main_launch)[0], [model_arg])]

# # for i in range(int(followers)):
# #     ns_arg = 'ns:=robot' + str(i)
# #     follower_launch = ['swarmbots', 'follower.launch']
# #     launch_files.append((roslaunch.rlutil.resolve_launch_arguments(follower_launch)[0], [model_arg, ns_arg]))

# print(launch_files)

# parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

# parent.start()

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['/my_ros_data/cosi_ws/src/swarmbots/launch/main.launch','model:=burger_for_autorace']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()