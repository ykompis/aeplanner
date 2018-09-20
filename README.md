# Autonomous Exploration Planner - aeplanner

## Assumptions
1. You are running `Ubuntu 16.04`
2. You are using bash, zsh is also possible with some [modifications](https://github.com/mseln/aeplanner/wiki/Use-zsh-instead-of-bash).
3. You have `ros-kinetic-desktop-installed`, otherwise follow installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
4. You have created a workspace in `~/catkin_ws`. If in another location [these](https://github.com/mseln/aeplanner/wiki/Other-location-than-~-catkin_ws) things need to be modified.

## System Requirements
This repository has been tested with: 
```
Ubuntu 16.04
ROS Kinetic (Desktop full installation)
catkin tools
catkin simple
OctoMap
```

it might work with other setups, but no guarantees are given.

## Setup dependencies

#### Catkin tools

Installation instructions [here](https://catkin-tools.readthedocs.io/en/latest/installing.html)

#### Setup workspace

If you do not already have a workspace, set up a new one.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin build
```

#### Octomap
```
sudo apt-get install "ros-kinetic-octomap-*"
```
or install from source.


#### Catkin simple
```
cd ~/catkin_ws/src
git clone git@github.com:catkin/catkin_simple.git
```

#### Install pigain dependencies
```
pip install rtree
sudo apt-get install libspatialindex-dev
```

## Setup aeplanner

Clone this package (make sure that you are in catkin_ws/src)
```
cd ~/catkin_ws/src/
git clone git@github.com:mseln/aeplanner.git
cd ..
catkin build
```

## Simulation environment

To run the example directly without having to integrate the interface, the kth uav simulation package can be installed, see:

```
https://github.com/danielduberg/kth_uav
```

Read the Assumptions and follow the install instructions on that page.

#### Troubleshooting installation of kth_uav

If you get `Could not find a package configuration file provided by "geographic_msgs" with any of the following names:` install it by running:
```
sudo apt-get install ros-kinetic-geographic-msgs
```

If you get `ImportError: No module named future` install it by running:
```
pip install future
```
