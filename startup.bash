#!/bin/bash

# Note: effect of "export DISPLAY=host.docker.internal:0.0" on Linux systems is unknown 
colcon build && 
source install/setup.bash &&
export DISPLAY=host.docker.internal:0.0 &&  
ros2 launch urdf_dummy sim.launch.py
