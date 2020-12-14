---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: default
---

# Introduction

## Watch it in action!

<iframe width="560" height="315" src="https://www.youtube.com/embed/34NjbfXCVM0" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

![Swarmbots following each other](images/swarmbots_follow.gif)
![Swarmbots mapping](images/swarmbots_mapping.gif)

## Problem statement
The original goal was to allow multiple robots in a swarm to collaboratively perform a single task. We planned to designate a swarm leader (aka *swarmboss*) and have the other robots in the simulation follow the leader.

As we worked on the project, our objectives changed slightly. We realized that certain tasks, such as SLAM, benefit more when robots aren't necessarily following the leader. Ideally, each robot would instead be mapping a different area of the environment, so that the merged map could be completed more quickly. Therefore we came up with a more complex algorithm for non-leader robots in the swarm, where they can either follow the leader or disperse from each other, improving the mapping time significantly.

Another objective which changed was the process of designating a leader for the swarm. We originally thought that we could choose the leader from the beginning and have that robot remain the leader for the entire simulation. However, robots are unpredictable, and we found that depending on the complexity of the world, they could sometimes get stuck. We realized that if the leader got stuck, this could be detrimental to the swarm as a whole. So, we designed an algorithm for switching swarm leaders at any time. 

## Relevant literature

### [Sparse Robot Swarms: Moving Swarms to Real-World Applications](https://www.frontiersin.org/articles/10.3389/frobt.2020.00083/full?fbclid=IwAR0A9xBr4wVydrBDofXauRAnFr9zKo1CM1nSaVqZFURWm1iMGi0iCAAVbC4)
While we didn't directly use the information from this article in our code, this gave us some background information on the uses and limitations of robot swarms in the real world.

### [How to Launch Multiple Robots in Gazebo](https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/)
This article helped us get started on launching multiple robots in the project. 
to suit our specific needs, we diverged significantly from their method of launching multiple robots; however, many of the basics remained the same, and it served as a very useful starting point.

### [Dynamic Launch Files - ROS Answers](https://answers.ros.org/question/229489/how-do-i-create-dynamic-launch-files/)
While developing the project, we decided that we wanted to be able to specify the desired number of robots as a command-line argument. To do that, the nodes started by the launch file had to change depending on the value of that argument. This ROS Answers thread gave us some important clues on creating dynamic launch files.

### [multirobot_map_merge - Package Documentation](http://wiki.ros.org/multirobot_map_merge)
Part of our objective was for the swarmbots to collaborate on the mapping of a shared environment. After implementing SLAM on each individual robot, we used this package to merge the maps into a single shared map.

### Gazebo Docs: [Building a World](http://gazebosim.org/tutorials?tut=build_world) and [Population of Models](http://gazebosim.org/tutorials?tut=model_population)
We used the information in these tutorials to build a custom Gazebo world to use for the swarmbots environment. The first tutorial is a basic overview of creating a Gazebo world and adding, removing, and editing models. The second tutorial shows how to create a population of models, or a group of identical models with a specific distribution across a region.
