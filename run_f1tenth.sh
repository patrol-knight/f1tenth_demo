#!/bin/bash
source install/setup.bash

ros2 launch f1tenth_launch f1tenth.launch.xml \
    map_yaml:=$HOME/f1tenth_core/maps/tepper/map_0201.yaml \
    use_sim_time:=false \
    launch_rviz:=false \
    launch_system:=true \
    launch_localization:=true \
    launch_planning:=true \
    launch_control:=true \
    "$@"
  
