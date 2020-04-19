#!/bin/bash 

echo "Viridian=32.794229,-97.094320,143,0" >> ~/ardupilot/Tools/autotest/locations.txt
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
cp ardu-parms/gazebo-boat.parm ~/ardupilot/Tools/autotest/default_params/