# swarmbots
- run world and initial robots: `roslaunch swarmbots main.launch world:=stage_4 robots:=2`
- start mapping: `roslaunch swarmbots gmapping_merge.launch robots:=2`
- start move_base: `roslaunch swarmbots turtlebot3_navigation.launch robots:=2`
