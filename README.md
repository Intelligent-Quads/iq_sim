# iq_sim
example gazebo arducopter simulation package

```
sudo apt install x-term
```

## boat sim

[boat setup](boat_setup.md)

```
cd ~/catkin_ws/src/iq_sim/scripts
./boatSetup.sh
```

run sitl 
```
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --console -L Viridian
```
