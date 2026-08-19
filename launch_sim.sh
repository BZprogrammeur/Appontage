#!/bin/bash
WS=/home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ws_ros2
PX4=/home/PX4-Autopilot

export LD_LIBRARY_PATH=$WS/install/wind_msgs/lib:$LD_LIBRARY_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/.gz/sim/plugins

source /root/ros2_humble/install/setup.bash
source $WS/install/setup.bash

mkdir -p /root/.gz/sim/plugins
cp $WS/install/wind_plugin/lib/wind_plugin/libwind_plugin.so /root/.gz/sim/plugins/

cd /home

PX4_SYS_AUTOSTART=4010 \
PX4_SIM_MODEL=gz_x650_camera \
PX4_GZ_MODEL_POSE="2,2,1,0,0,0.9" \
PX4_GZ_WORLD=ocean \
$PX4/build/px4_sitl_default/bin/px4
