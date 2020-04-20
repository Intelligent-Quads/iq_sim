# Installing Ardupilot and MAVProxy

### Video tutorial at https://youtu.be/wlkoq65mM2A

## Clone ArduPilot

In home directory:
```
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

## Install dependencies:
```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk git python-pip python-pexpect
```

## Use pip (Python package installer) to install mavproxy:
```
sudo pip install future pymavlink MAVProxy
```

MAVProxy is a fully-functioning GCS for UAV’s. The intent is for a minimalist, portable and extendable GCS for any UAV supporting the MAVLink protocol (such as one using ArduPilot). For more information check out http://ardupilot.github.io/MAVProxy/html/index.html

Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:
```
. ~/.bashrc
```

Run SITL (Software In The Loop) once to set params:
```
sim_vehicle.py -v APMrover2
```

# Intalling QGroundControl 

### Video tutorial at https://youtu.be/qLQQbhKDQ6M

## Overview 

QGroundControl provides full flight control and vehicle setup for PX4 or ArduPilot powered vehicles. It provides easy and straightforward usage for beginners, while still delivering high end feature support for experienced users.

### Key Features:

- Full setup/configuration of ArduPilot and PX4 Pro powered vehicles.
- Flight support for vehicles running PX4 and ArduPilot (or any other autopilot that communicates using the MAVLink protocol).
- Mission planning for autonomous flight.
- Flight map display showing vehicle position, flight track, waypoints and vehicle instruments.
- Video streaming with instrument display overlays.
- Support for managing multiple vehicles.
- QGC runs on Windows, OS X, Linux platforms, iOS and Android devices.

for more detailed information please visit http://qgroundcontrol.com/

## install QGroundControl for Ubuntu Linux 16.04 LTS or later:

Download QGroundControl.AppImage 
```
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
```
change permissions and run 
```
chmod +x ./QGroundControl.AppImage 
./QGroundControl.AppImage  #(or double click)
```

## Run SITL and connect with Q Ground

```
sim_vehicle.py -v APMrover2
```

# Installing Gazebo and ArduPilot Plugin

### Video Tutorial at https://youtu.be/m7hPyJJmWmU

## Overview 

Robot simulation is an essential tool in every roboticist's toolbox. A well-designed simulator makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. At your fingertips is a robust physics engine, high-quality graphics, and convenient programmatic and graphical interfaces. Best of all, Gazebo is free with a vibrant community.

for more infromation on gazebo checkout http://gazebosim.org/

## Install Gazebo

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```
sudo apt update
```

Install Gazebo:
```
sudo apt install gazebo9 libgazebo9-dev
```

for more detailed instructions for installing gazebo checkout http://gazebosim.org/tutorials?tut=install_ubuntu

## Install Gazebo plugin for APM (ArduPilot Master):
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

## Run Simulator
In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

# Install ROS and Setup Catkin  

### Video Tutorial at https://youtu.be/EmIjedzHwzI

In this tutorial we are using **Ubuntu 18.04** and **ROS Melodic**

Code blocks are meant to be typed in Terminal windows. "Control+Alt+T" opens a new Terminal window.

## 1. Install ROS

   - Do _Desktop-full Install_
   - Follow until _Step 1.7_ at the end of the page

   First, install **ROS Melodic** using the following instructions: http://wiki.ros.org/melodic/Installation/Ubuntu


## 2. Set Up Catkin workspace

We use `catkin build` instead of `catkin_make`. Please install the following:
```
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

Then, initialize the catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

## 3. Dependencies installation

Install `mavros` and `mavlink` from source:
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables
```
source ~/.bashrc
```

## 4. Clone IQ Simulation ROS package 

```
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
cd iq_sim
git checkout boat
```
Our repository should now be copied to `~/catkin_ws/src/iq_sim/` (don't run this line. This is just saying that if you browse in the file manager, you will see those folders).

## 5. Build instructions
   Inside `catkin_ws`, run `catkin build`:

```
cd ~/catkin_ws
catkin build
```
update global variables
```
source ~/.bashrc
```
## 6. Setup libgazebo_usv_dynamics_plugin

The boat simulator uses the usv dynamics plugin, which can be compiled from the px4 sitl_gazebo repo.




## *sitl_gazebo* plugin dependencies

Some plugins on this packages require some specific dependencies:

* Protobuf is required to generate custom protobuf messages to be published and subscribed between topics of different plugins;
* Jinja 2 is used to generate some SDF models from templates;
* Gstreamer is required for a plugin that streams video from a simulated camera.


### Ubuntu 

```bash
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
```

#### Gstreamer:
```
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
```


### Mac OS

```bash
pip install rospkg jinja2
brew tap homebrew/versions
brew install eigen graphviz libxml2 sdformat3 opencv glib
brew install homebrew/versions/protobuf260
```

#### Gstreamer:
```
brew install gstreamer gst-plugins-base gst-plugins-good
```

### Arch Linux

```bash
sudo pacman -S --noconfirm --needed eigen3 hdf5 opencv protobuf vtk yay python2-jinja
```

#### Gstreamer:
```bash
sudo pacman -S --needed gstreamer gst-plugins-bad gst-plugins-base gst-plugins-base-libs gst-plugins-good gst-plugins-ugly
```


## Build *sitl_gazebo*

Clone the repository to your computer.

**IMPORTANT: If you do not clone to ~/src/sitl_gazebo, all remaining paths in these instructions will need to be adjusted.**

```bash
mkdir -p ~/src
cd src
git clone --recursive https://github.com/PX4/sitl_gazebo.git
```



Navigate into the build directory and invoke CMake from it:

```bash
cd ~/src/sitl_gazebo
mkdir build
cd build
cmake ..
```

Now build the gazebo plugins by typing:

```bash
make -j$(nproc) -l$(nproc)
```

Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your `.bashrc` (Linux) or `.bash_profile` (Mac) file:

```bash
# Set the plugin path so Gazebo finds our plugins
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/build
```

You also need to add the the root location of this repository, e.g. add the following line to your `.bashrc` (Linux) or `.bash_profile` (Mac) file:

```bash
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$HOME/src/sitl_gazebo
```

## Install

If you wish the libraries and models to be usable anywhere on your system without
specifying th paths, install as shown below.

**Note: If you are using Ubuntu, it is best to see the packaging section.**

```bash
sudo make install
```


## 7. Add Simulator Specific Variables

```
gedit .bashrc
```
add to the bottom 
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models
```
save and close 
```
source ~/.bashrc
```

## 8. Setup IQ GNC ROS package

```
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_gnc.git
cd iq_gnc
git checkout boat
catkin build
```

## 9. add custom location to ardurover sitl 
```
cd ~/catkin_ws/src/iq_sim/scripts/
./boatSetup.sh
```

## 10. Run Boat Sim

First terminal 
```
roslaunch iq_sim boat.launch
```
second terminal
```
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --console -L Viridian
```
on the first run, run the below in the mavproxy terminal
```
param load catkin_ws/src/iq_sim/scripts/ardu-parms/gazebo-boat.parm 
```

thrid terminal 
```
roslaunch iq_sim apm.launch
```
fourth terminal
```
rosrun iq_gnc square
```

set mode to guided by running the below in the mavproxy terminal (second terminal)
```
mode GUIDED
```

boat should drive a square pattern
