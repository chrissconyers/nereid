#!/bin/bash

. /opt/ros/eloquent/setup.bash
ros2 run rviz2 rviz2 -d config/dipc.rviz &
./install/bin/dipc_viz
