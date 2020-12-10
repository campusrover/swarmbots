---
layout: default
title:  "What was created"
---

# What was created

## Technical descriptions, illustrations
![swarmbots world](/images/gazebo_cap_1.jpg)
A picture of the gazebo world built for the robots. Pictured are three turtlebot3 burger models inside a maze-like area with brown walls. The blue cylinders denote charging stations for the robots.

![tf tree diagram](/images/tf_demo.png)
This diagram shows an example of a tf tree when three robots are run at once. Each robot’s tree starting from robot#/odom has a parent frame connecting to its map. The maps in turn are children of the world frame, which is a child of the merged map at the tf frame “map”.
Should the robots not require mapping, the tf tree is also modular--if the robot#/map topic disappears, the world frame will have “robot#/odom” as children instead.

{merged map examples}
![stage_4 mapped](/images/stage_4_fully_mapped.png)
![swarmbots_sm mapped](/images/swarm_sm_fully_mapped.png)
![gazebo swarmbots_sm screenshot](/images/gazebo_swarmbots_sm.jpg)

{state diagram}

## Discussion of interesting algorithms, modules, techniques

### State switching
Each swarmbot consists of the same basic states, and its actions depend on the state that it is in. {include State diagram? maybe link to images section?} If it is leading, it will perform the leader’s actions. If it is following, it will perform the follower instructions. If it is dead (stuck), it will attempt to recover. If it cannot perform a higher level instruction because of an obstacle it will return to wandering. {expand upon with the one paper we read in class}. For a swarmbot to find the statuses of the other swarmbots, a `/state` topic was required that each robot publishes messages to, which includes information on if a swarmbot is leading, following, or dead. Because a non-leader swarmbot's behavior depends on the leader's behavior, each swarmbot keeps track of the current leader. If a leader detects that it has died, it will assign a new leader by publishing to the `/state` topic. In addition, if a leader stops publishing its state altogether, a new leader is chosen automatically after several seconds.

### Follow/disperse algorithm
The original problem statement was to apply the swarmbots to different sets of tasks. To adhere to the problem statement, the swarmbots were designed to be able to perform different commands. Currently the two commands implemented are follow and disperse.
- follow: follower swarmbots follow the leader from a safe distance
- disperse: follower swarmbots separate from the leader at least some number of meters apart, and then explore the rest of the world
By building upon the basic set of commands more complex problems can be solved. For instance, a swarm of robots inside a maze could disperse and try multiple paths--when a path out of the maze is found by one robot, it becomes the leader that the rest of the robots would then follow them to navigate out of the maze. Similar problems requiring traversal of large areas would also apply.

### Dynamic launch
Rather than hard-coding the number of robots in a swarm, we wanted to be able to specify the size of the swarm as a command-line argument. This way, we could handle environments of different sizes and complexities just by changing the value of the argument, rather than copying and pasting code in the launch files. Because each robot requires several nodes to be running in its namespace, we needed a launch file with something like a loop. Since loops don't exist in XML launch files, we instead used a combination of `eval` and `if` to recursively spawn another robot while the `if` condition evaluated to true. The `if` condition relied on the number of robots remaining to be greater than 0, so in each recursive call, that number was decremented.

## Problems that were solved, pivots that had to be taken

### Dynamically launching multiple robots
The dynamic launch file proved to be more of a challenge than we were expecting. At first, we tried to use the [python roslaunch API](http://wiki.ros.org/roslaunch/API%20Usage). In theory, a python script can be used in place of an XML launch file. Through different calls to `rospy`, the script can launch multiple files and nodes, while benefiting from all the features of python (loops, functions, lists, and complex expressions). However, we were plagued with bizarre error messages, and while every ROS Answers thread seemed to have a different solution, none of them ended up working. Then, we tried to launch the necessary files using a bash script. Anything that can be done through the command line can also be done in bash, including loops and calls to `roslaunch`. However, executing the bash script took far longer than we expected - much longer than launching a single file that included other launch files and nodes. In the end, we settled with the recursive approach.

### Switching the leader at any time
Sometimes, robots could get stuck in the Gazebo world and be unable to move. Because the movement of the non-leader swarmbots depend on the leader swarmbot, the leader getting stuck would be detrimental to the entire swarm. Thus, we needed to ensure a smooth transition of power from swarmbot to swarmbot, in case a leader got stuck or died. Our code was originally written so that the leader would be assigned at launch and never change. So, the nodes running on the leader robot were different than the nodes running on the non-leader robots. However, if we wanted to be able to switch leaders at any time, then we needed all of the swarmbots to be able to run both leader and non-leader code. This required reformatting what was the non-leader code into a state machine structure, which included a "lead" state. When necessary, any non-leader could then become the leader by switching its state to "lead".

### Tf trees for mapping {tf frames to maps}
The original map topics in gmapping for each robot# namespace set the tf frame of the map as the parent of the odom frame, which would overwrite the world frame created to connect the tf frames between the robots. To work around this, the broadcaster for the world frame would check each time before being published if the maps’ frames existed, and parent the maps instead. A similar problem arose with map_merge whose package does not publish a tf frame for the map thus a broadcaster had to be implemented by hand.

### Usage of robot initial positions {usage of initial positions}
While we initially hoped for each robot to be able to localize itself in an area with AMCL and thus decrease the need for accurate odometry, our goal was focused on the behaviors of the swarm itself as well as completion of the mapping task, and due to limited time usage of AMCL had to be set aside. In addition, the limitations of the launch files make it so that we had to designate spawn positions for each robot. However localization is entirely feasible (call map_merge with initial positions set to false, and use AMCL package on merged map) should an expansion upon the project happen.

{move_base time constraints, dunno if this part should be kept}
The other feature that had to be set aside due to time constraints was implementing move_base for the namespaced robots. This would have made it such that robots would plan a path back to the leader while avoiding obstacles.
