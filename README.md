# SWARMBOTS

Visit the [website](https://campusrover.github.io/swarmbots) for more details about SWARMBOTS.

## Table of Contents

* [Collaborators](#collaborators)
* [Description](#description)
* [Tech stack](#tech-stack)
# [Installation](#installation)
# [Usage](#usage)

## Collaborators

| Name | Github |
| --- | --- |
| Kelly Duan | [celry](https://github.com/celry) |
| Evalyn Berleant | [eberleant](https://github.com/eberleant) |

## Description

SWARMBOTS allows multiple robots in a swarm to collaboratively map their environment. At any time, there is a single leader (the *swarmboss*) whose movement dictates the behavior of the other robots in the swarm, who may follow or disperse from the leader.

## Tech stack

* ROS melodic
* Gazebo 9.0.0
* Python 2.7.17
* [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge)

## Installation

1. Clone the repo into catkin_ws/src: `git clone https://github.com/campusrover/swarmbots.git`
1. Clone map merge into catkin_ws/src: `git clone https://github.com/hrnr/m-explore.git`
1. Run `catkin_make` from the catkin_ws directory

## Usage
- run world and initial robots: `roslaunch swarmbots main.launch world:=stage_4 robots:=2`
  - change world by setting `world:=[world name in world folder]`
    - recommended worlds: `stage_4`, `swarmbots`
  - change number of robots by setting `robots:=[# of robots]`
- toggle swarm orders: `rosrun swarmbots command.py`
- view map in rviz:
  - launch `rviz`
  - set Fixed Frame to `world`
  - Add by topic `/map`
- save maps for future use: `roslaunch swarmbots savemap.launch robots:=2`
  - save maps of robots running and the merged map
  - change number of robots by setting `robots:=[# of robots]`
