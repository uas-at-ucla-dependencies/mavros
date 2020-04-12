MAVROS (with ROS2) setup for MacOS
======

## Prerequisite
- You do NOT need to clone this repository
- ROS2 (recommended version is Eloquent): https://index.ros.org/doc/ros2/Installation/

## Setup & Installing tools
Mavros is built using colcon. More info here: https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/
```bash
python3 -m pip install colcon-common-extensions # if it complains about unsupported versions, `pip install --upgrade` the offending packages
python3 -m pip install vcstool
python3 -m pip install future

curl -sL https://gist.githubusercontent.com/dendisuhubdy/19482135d26da86cdcf442b3724e0728/raw/7c15d65789c794de84006d648dccbbe99c167399/endian.h > /usr/local/include/endian.h # messy hack, but it works
```

## Building mavros
```bash
source ~/ros2_eloquent/ros2-osx/setup.zsh # get ros environment variables set up
mkdir mavros_ws
cd mavros_ws
mkdir src

curl -sL https://gist.githubusercontent.com/RyGuy101/487fd32f00acbfb53c0c7d118a825d0c/raw/0a03017552a838a59b96634ef4a72c8cfee6267f/mavros.repos > mavros.repos # fetch list of repositories needed to build mavros

vcs import src < mavros.repos --recursive # fetch all repositories
colcon build --packages-up-to mavros # build mavros and all dependent packages using colcon
```
