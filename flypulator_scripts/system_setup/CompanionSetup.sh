#!/bin/bash
# This script installs Ros Melodic and gazebo 9.0 on Ubuntu 18.04
echo "Setup to accept packages from packages.ros.org"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "Set up keys"
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update
echo "installing Ros desktop full"
sudo apt-get -y install ros-melodic-desktop-full

echo "initialize and update ROSDEP"
sudo rosdep init
rosdep update

#echo "setting up environment for bash"
#echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
#to install mavros
sudo apt install mavros mavros-extras 
                                      
# for geographiclib:                    
sudo apt install geographiclib-tools geographiclib-doc
                                          
#install datasets:                     
sudo geographiclib-get-magnetic emm201
sudo geographiclib-get-gravity egm96  
sudo geographiclib-get-geoids egm96-5 

echo "install dependencies for building packages"
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
