---
layout: default
title:  "Project reflection"
---

# Project reflection

## How the team worked together
We held extensive meetings at least 2-3 times a week to discuss our objectives, write code, and complete the project deliverables. While we never had conflicts over which sections of the project we wanted to code, both of us occasionally got stuck on a problem. When this happened, we were usually able to talk to each other and solve the problem together. If we weren't able to solve the problem, then we agreed to shelve it until later. Because of our mutual commitment to the project, we had a steady rate of progress each week from start to finish.

## Reflection on the project
Overall, we worked together and communicated effectively as a team. In the process, we learned a lot about ROS, particularly writing launch files, namespaces, transforms, and mapping. We also built on our knowledge of messaging by creating a custom message type. While we brought the project to a respectable level of completion, there are still some features that we think could help in addressing our goal.

## Ideas for improvement

### More sophisticated navigation
At the moment, each swarmbot's navigation abilities are quite basic, no matter which state it is in. The leader moves in a roomba-like pattern, while non-leaders can get stuck fairly frequently in either the `follow` or `disperse` state. More sophisticated navigation and path-planning would be a great improvement to the project. Ideally, we could integrate the ROS navigation stack with our state-based code, allowing each robot to plan low-cost paths using the merged map.

### Autonomous survival
An interesting addition to the project would be creating a more complex gazebo world with various "resources". The robots could not only collaborate on the overall world map, but also work together to create an additional resource map. The robots would be required to constantly have enough resources in order to stay alive. Ideally, robots could try to survive together by creating and stocking bases. It could be like an autonomous robotics version of a survival game.

### Purposeful mapping
The project could be greatly improved by analyzing the merged map to find areas that have not yet been mapped. Then, robots could visit these specific areas, which would speed up and streamline the mapping process.

### More complex world
Adding additional types of obstacles would give the swarmbots an extra challenge. In a more complicated Gazebo world, we could add moving obstacles or obstacles that randomly spawn or disappear. Different terrain or ground altitudes could also challenge the robots’ navigation.

### GUI
Adding a GUI that updates in real-time, reflecting what is happening in the swarmbots world, would be both a nice improvement and useful for debugging purposes. 
