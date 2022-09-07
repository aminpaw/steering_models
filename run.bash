#!/bin/bash 
rosrun turtlesim turtlesim_node &
sleep 1;
rosservice call /spawn "x: 5.544445
y: 5.544445
theta: 0.0
name: ''"
rosservice call /turtle1/set_pen "{r: 250, g: 0, b: 0, width: 3, 'off': 0}"
rosservice call /turtle2/set_pen "{r: 250, g: 250, b: 250, width: 3, 'off': 0}"
source devel/setup.bash

