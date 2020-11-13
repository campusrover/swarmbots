#!/usr/bin/env python
import rospy, roslaunch, sys

def analyze_args(cli_args):
    parsed = dict()
    for arg in cli_args:
        if ':=' in arg:
            idx = arg.index(':=')
            parsed[arg[:idx]] = arg[idx+2:]
    return parsed

parsed_args = analyze_args(sys.argv)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

model_arg = 'model:=' + parsed_args.get('model', 'burger_for_autorace')
followers = parsed_args.get('followers', 2)

main_launch = ['swarmbots', 'main.launch']
launch_files = [(roslaunch.rlutil.resolve_launch_arguments(main_launch)[0], [model_arg])]

for i in range(int(followers)):
    ns_arg = 'ns:=robot' + str(i)
    follower_launch = ['swarmbots', 'follower.launch']
    launch_files.append((roslaunch.rlutil.resolve_launch_arguments(follower_launch)[0], [model_arg, ns_arg]))

print(launch_files)

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()