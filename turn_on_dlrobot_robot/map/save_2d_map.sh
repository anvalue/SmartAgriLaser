#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/dlrobot/dlrobot_robot/devel/setup.bash
rm -rf /home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/map/DLROBOT.pgm 
rm -rf /home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/map/DLROBOT.yaml
rosservice call /finish_trajectory 0
rosservice call /write_state /home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/map/map.bag.pbstream
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/map/map.bag.pbstream

cp  map.pgm DLROBOT.pgm

cp  map.yaml DLROBOT.yaml

