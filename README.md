# SWARMBOTS

Install map merge: `git clone https://github.com/hrnr/m-explore.git`

- run world and initial robots: `roslaunch swarmbots main.launch world:=stage_4 robots:=2`
  - change world by setting `world:=[world name in world folder]`
    - recommended worlds: `stage_4`, `swarmbots`
  - change robots by setting `robots:=[# of robots]`
- toggle swarm orders: `rosrun swarmbots command.py`
- view map in rviz:
  - launch `rviz`
  - set Fixed Frame to `world`
  - Add by topic `/map`
