#!/bin/bash

export ROS_MASTER_URI=http://localhost:11311
source /home/jetauto/jetauto_workshop/devel/setup.sh

rosrun jetauto_movement move.py forward 6
rosrun jetauto_movement move.py turnleft 4
rosrun jetauto_movement move.py forward 4
rosrun jetauto_movement move.py turnleft 4
rosrun jetauto_movement move.py forward 3
rosrun jetauto_movement move.py turnright 4
rosrun jetauto_movement move.py forward 4
rosrun jetauto_movement move.py turnright 4
rosrun jetauto_movement move.py forward 4
rosrun jetauto_movement move.py turnleft 4
rosrun jetauto_movement move.py forward 4




