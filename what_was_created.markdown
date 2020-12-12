---
layout: default
title:  "What was created"
---

# What was created

## Technical descriptions, illustrations

![swarmbots world](images/gazebo_cap_1.jpg)
A picture of the gazebo world built for the robots. Pictured are three turtlebot3 burger models inside a maze-like area with brown walls. The blue cylinders represent charging stations for the robots.

![tf tree diagram](images/tf_demo.png)
This diagram shows an example of a tf tree when three robots are run at once. Each robot’s tree starting from `robot#/odom` has a parent frame connecting to its map. The maps in turn are children of the world frame, which is a child of the merged map at the tf frame `map`.
Should the robots not require mapping, the tf tree is also modular--if the `robot#/map` topic and respective tf disappears, the world frame will have `robot#/odom` as its children instead.

#### Merged map examples

![stage_4 mapped](images/stage_4_fully_mapped.png)

![swarmbots_sm mapped](images/swarm_sm_fully_mapped.png)

![gazebo swarmbots_sm screenshot](images/gazebo_swarmbots_sm.jpg)

![Launch file diagram](images/launch_tree.png)

This diagram shows the layout of the launch files in the package, which are separated for modularity. The root file is at `main.launch` which starts the gazebo world, `gmapping_merge`, and the `follower` launch files. The arrows in `gmapping` and `follower` indicate that the launch files are run multiple times, once for each robot. The `savemap.launch` file is disconnected from the main file so that the user can manually save after SLAM is given enough time to generate the maps.

## Discussion of interesting algorithms, modules, techniques

### State switching
Each swarmbot consists of the same basic states, and its actions depend on the state that it is in. If it is in a leading state, it will perform the leader’s actions. If it is following, it will perform the follower’s actions. If it is dead (stuck), it will attempt to recover. If it cannot perform a higher level action because of an obstacle it will return to wandering (as with Brooks - Robust Layered Control System for a Mobile Robot). For a swarmbot to find the statuses of the other swarmbots, a `/state` topic was required that each robot publishes messages to, which includes information on if a swarmbot is leading, following, or dead. Because a non-leader swarmbot's behavior depends on the leader's behavior, each swarmbot keeps track of the current leader. If a leader detects that it has died, it will assign a new leader by publishing two messages to the `/state` topic: one that declares itself dead, and a second that declares another robot the leader. In addition, if a leader stops publishing its state altogether, a new leader will be chosen automatically after several seconds.

### Follow/disperse algorithm
The original problem statement was to apply the swarmbots to different sets of tasks. To adhere to the problem statement, the swarmbots were designed to be able to perform different commands. Currently the two commands implemented are follow and disperse.
- follow: follower swarmbots follow the leader from a safe distance
- disperse: follower swarmbots separate from the leader at least some number of meters apart, and then explore the rest of the world
By building upon a basic set of commands more complex problems can be solved. For instance, a swarm of robots inside a maze could disperse and try multiple paths--when a path out of the maze is found by one robot, it becomes the leader that the rest of the robots would then follow them to navigate out of the maze. Similar problems requiring traversal of large areas would also apply.

### Dynamic launch
Rather than hard-coding the number of robots in a swarm, we wanted to be able to specify the size of the swarm as a command-line argument. This way, we could handle environments of different sizes and complexities just by changing the value of the argument, rather than copying and pasting code in the launch files. Because each robot requires several nodes to be running in its namespace, we needed a launch file that could loop on itself. Since loops don't exist in XML launch files, we instead used a combination of `eval` and `if` to recursively spawn another robot while the `if` condition evaluated to true. The `if` condition relied on the number of robots remaining to be greater than 0, so in each recursive call, that number was decremented.

## Problems that were solved, pivots that had to be taken

### Dynamically launching multiple robots
The dynamic launch file proved to be more of a challenge than we were expecting. At first, we tried to use the [python roslaunch API](http://wiki.ros.org/roslaunch/API%20Usage). In theory, a python script can be used in place of an XML launch file. Through different calls to `rospy`, the script can launch multiple files and nodes, while benefiting from all the features of python (loops, functions, lists, and complex expressions). However, we were plagued with bizarre error messages, and while every ROS Answers thread seemed to have a different solution, none of them ended up working. Then, we tried to launch the necessary files using a bash script. Anything that can be done through the command line can also be done in bash, including loops and calls to `roslaunch`. However, executing the bash script took far longer than we expected - much longer than launching a single file that included other launch files and nodes. In the end, we settled with the recursive approach, which we described in the previous section.

### Switching the leader at any time
Sometimes, robots could get stuck in the Gazebo world and be unable to move. Because the movements of the non-leader swarmbots depend on the leader swarmbot, the leader getting stuck would be detrimental to the entire swarm. Thus, we needed to ensure a smooth transition of power from swarmbot to swarmbot, in case a leader got stuck or died. Our code was originally written so that the leader would be assigned at launch and never change. So, the nodes running on the leader robot were different from the nodes running on the non-leader robots. However, if we wanted to be able to switch leaders at any time, then we needed all of the swarmbots to be able to run both leader and non-leader code. This required rewriting what was the non-leader code into a state machine structure, which included a "lead" state. When necessary, any non-leader could then become the leader by switching its state to "lead".

### Spawning with random positions
One problem we were stuck on was using parameters with shared values inside our tree of launch files. Originally, we relied mostly on passing args between launch files. However, spawning robots in randomized positions would require *parameters* to generate the random numbers, instead of args. For this case, we had to directly change the spawn_model python file from the gazebo_ros melodic build such that it would accept parameters for the spawning positions, as before it would only accept a line of arguments as the spawn position like so:
``` xml
<node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model $(arg robot_name) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param /robot_description" />
```
In addition to that, map_merge requires use of parameters set for each namespace to define the initial position. Thus we created a new script to not only generate random values for our `x` and `y` positions, but also overwrite the parameters required for map_merge.

### Tf trees for mapping
The original map topics in gmapping for each `robot#` namespace set the tf frame of the map as the parent of the robot's odom frame, which would overwrite the world frame created to connect the tf frames between the robots. To work around this, the broadcaster for the world frame would check each time before being published if the maps’ frames existed, and parent the maps instead. A similar problem arose with map_merge, whose package does not publish a tf frame for the map, so a broadcaster had to be implemented by hand.

### Usage of robot initial positions and odometry
We initially hoped for each robot to be able to localize itself on the merged map using AMCL and without knowing its initial position, decreasing the need for accurate odometry estimations. However, we deemed map merging through feature matching and the amcl package too inaccurate and slow to implement. Localization is entirely feasible (call map_merge with initial positions set to false, and use AMCL package on merged map) should an expansion upon the project happen.

### Move_base and time constraints
The other feature that had to be set aside due to time constraints was implementing move_base with correctly generated costmaps for the namespaced robots. This would have made it such that robots would plan a path back to the leader while avoiding obstacles.
