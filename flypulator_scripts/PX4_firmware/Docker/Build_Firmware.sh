#!/bin/bash

# This Docker image can be used to build the firmware and to run ROS/Gazebo with SITL for PX4.
# It is assumed that the PX4 code is located in ~/src on the local machine.

# to find dns of uni network type:$ nmcli dev show | grep 'IP4.DNS'
# In order to make gazebo run on host the following updates were done:
# run script with sudo, ie sudo ./run_docker.sh
# within the Container:
# apt-get update
# apt-get upgrade (dont know if its necessary)
# apt-get install ubuntu-drivers-common // to get next command..
# ubuntu-drivers devices // look for matching driver with host...
# // my was nvidia-driver-396. if not found proceed
# apt-get install software properties-common //to add repository
# add-apt-repository ppa:graphics-drivers/ppa
# apt-get update
# ubuntu-drivers devices // see if correct driver is listed as recommended
# ubuntu-drivers autoinstall
# gazebo //test if gazebo launches on host


# to Build for all targets:
# cd src/firmware // ( within docker container )
# make px4fmu_firmware
echo "Spinning up docker container for PX4 SITL with ROS & Gazebo"




#    --name=px4container px4io/px4-dev-ros:2018-07-19 bash
xhost +
printf "Access from container to xhost enabled\n"
docker run -it --privileged \
    --dns 141.30.1.1 \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v ~/src/Firmware:/src/firmware/:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro  \
    -e DISPLAY=:1 \
    -p 14556:14556/udp \
    --name=Firmwarebuild px4io/px4-dev-nuttx:2018-08-05 bash
