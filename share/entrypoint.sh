#!/bin/env bash
ROS_WS=$HOME/ros_ws

local_setup_bash=${ROS_WS}/install/setup.bash
if [[ -f $local_setup_bash ]]
then
    rcfile=$local_setup_bash
else
    rcfile=/opt/ros/foxy/setup.bash
fi

cd $ROS_WS
exec bash --rcfile $rcfile
