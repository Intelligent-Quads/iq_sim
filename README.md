# IQ Simulations

This repo hosts gazebo worlds for various drone scenarios and various drone configurations. This Repo is specifically designed to work with the Ardupilot control system, and utilizes the ardupilot gazebo plugin to allow the ardupilot control software to interface and control the model drone in gazebo. 

## IQ Tutorials
This Repo is part of the Intelligent Quads Software Development tutorial series. Please take a look at the following resources to learn more about how to use this repository.

[Intelligent Quads Text Tutorials](https://github.com/Intelligent-Quads/iq_tutorials)

[Intelligent Quads Video Tutorials](https://www.youtube.com/channel/UCuZy0c-uvSJglnZfQC0-uaQ)


## Dependencies 

take a look at these tutorials to setup ardupilot, gazebo and the ardupilot gazebo plugin 

[Installing Ardupilot and MAVProxy](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/Installing_Ardupilot.md)

[Installing QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_qgc.md)

[Installing Gazebo and ArduPilot Plugin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_gazebo_arduplugin.md)

Installing x-term is recommended as it allows the ardupilot sitl interface to run in a terminal that will cleanly close when closing you sitl instance
```
sudo apt install x-term
```

## Drone Simulations 

![runway world](docs/imgs/runway.jpg)

This repo contains a couple different gazebo worlds containing various ardupilot drone configurations. The worlds are listed below 

- `droneOnly.world` - simple gazebo world containing only a single drone
- `runway.world` - simple gazebo world containing only a single drone on a runway
- `lidar.world` - a simple gazebo world containing a single drone with a 2d lidar sensor
- `multi_drone.world` - a simple world containing 12 drones  

### Running Drone Simulations 

Each world contains a corresponding launch file. For example to launch `runway.world` run
```
roslaunch iq_sim runway.launch
``` 
Launch the ardupilot instance by running 
```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
``` 
For more information, take a look at the corresponding tutorials [here](https://github.com/Intelligent-Quads/iq_tutorials)

## Boat Simulation

Recently I have been experimenting with Ardurover for controlling an autonomous boat. Bellow shows the steps for installing for boat simulation. If you have already installed for quadcopter simulation, then most of the boat installation should already be completed. 

[boat setup](docs/boat_setup.md)

![river world](docs/imgs/river_world.jpg)

## Run Boat Sim 

First terminal 
```
roslaunch iq_sim boat.launch
```
second terminal
```
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --console -L Viridian
```
