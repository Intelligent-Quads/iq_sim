# IQ Simulations

This repo hosts gazebo worlds for various drone scenarios and various drone configurations. This Repo is specifically designed to work with the Ardupilot control system, and utilizes the ardupilot gazebo plugin to allow the ardupilot control software to interface and control the model drone in gazebo.  

## Dependencies 

take a look at these tutorials to setup ardupilot, gazebo and the ardupilot gazebo plugin 

[Installing Ardupilot and MAVProxy](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/Installing_Ardupilot.md)

[Installing QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_qgc.md)

[Installing Gazebo and ArduPilot Plugin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_gazebo_arduplugin.md)

Installing x-term is recommended as it allows the ardupilot sitl interface to run in a terminal that will cleanly close when closing you sitl instance
```
sudo apt install x-term
```

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

## Swarm Boats

```
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --console -L Viridian -I0 --out=tcpin:0.0.0.0:8100 
```

```
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --console -L Viridian -I1 --out=tcpin:0.0.0.0:8110 
```
