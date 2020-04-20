#!/bin/bash 

if cat ~/ardupilot/Tools/autotest/locations.txt | grep "Viridian=32.794229,-97.094320,143,0"
then
echo "location exist already"
else
echo "Viridian=32.794229,-97.094320,143,0" >> ~/ardupilot/Tools/autotest/locations.txt
fi
