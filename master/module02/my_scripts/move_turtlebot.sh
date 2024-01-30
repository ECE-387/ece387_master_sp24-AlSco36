#!/bin/bash

ARG1=$1

if [ $ARG1 == 'forward' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.15
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: 0.0"

elif [ $ARG1 == 'rotate' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: 0.5"

elif [ $ARG1 == 'stop' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: 0.0"
else
echo "Please enter one of the following:
    forward
    rotate
    stop"
fi
